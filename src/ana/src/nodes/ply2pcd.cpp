#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include "cuda_operations.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <fstream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <algorithm>  // Make sure to include this header for std::min and std::max


// FILTERS .PCD FROM RTAB MAP
// PUBLISHES
//  - mapped_point_cloud
//  - occupancy
//  - height_map

// cuda kernels
extern "C" void processPointCloudVoxelGrid(PointXYZ *hostPoints, int numPoints, float voxelSize);

class PCDPublisher : public rclcpp::Node {
public:
    PCDPublisher() : Node("pcd_publisher") {
        auto qos_default = rclcpp::QoS(rclcpp::KeepLast(100));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_point_cloud", qos_default);
        
        auto qos_grid = rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable();
        grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap_new", qos_grid);
        
        auto qos_height = rclcpp::QoS(rclcpp::KeepLast(100));
        height_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("height_map", qos_height);

        // Immediately invoke the publishing sequence
        publishOnce();
    }

private:
    void publishOnce() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/aidan/ana_bot/src/ana/rtab_maps/livox_map.pcd", *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the pcd file");
            return;
        }

        // RANSAC Plane Segmentation - PCL GPU?
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (inliers->indices.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model for the given dataset.");
        } else {
            // Extract the plane and non-plane parts
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false); // true - removes plane
            extract.filter(*plane_cloud); // Extracted plane points
            
            extract.setNegative(true);
            extract.filter(*non_plane_cloud); // Rest of the points

            // Apply Voxel Grid Downsampling to planes
            pcl::VoxelGrid<pcl::PointXYZ> voxelFilterPlane;
            voxelFilterPlane.setInputCloud(plane_cloud);
            voxelFilterPlane.setLeafSize(0.05f, 0.05f, 0.05f); // Increase these values to make the plane less dense
            voxelFilterPlane.filter(*plane_cloud);

            // Apply Voxel Grid Downsampling to non-planes - PCL GPU
            pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
            voxelFilter.setInputCloud(non_plane_cloud);
            voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f); // Increase these values to make the non-floor less dense
            voxelFilter.filter(*non_plane_cloud);
    
        }
        // *non_plane_cloud += *plane_cloud;

        // Combine the plane and non-plane points

        cloud.swap(non_plane_cloud); // Now cloud contains both sparsely sampled plane and other points

        pcl::PassThrough<pcl::PointXYZ> heightFilter;
        heightFilter.setInputCloud(cloud);
        heightFilter.setFilterFieldName("z");
        heightFilter.setFilterLimits(0.0, 0.20);
        heightFilter.filter(*cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = this->now();


        auto grid = createOccupancyGrid(cloud, 0.05f);  // Example: 2mm grid cell size
        for (const auto& point : cloud->points) {
            int x = static_cast<int>((point.x - grid.info.origin.position.x) / grid.info.resolution);
            int y = static_cast<int>((point.y - grid.info.origin.position.y) / grid.info.resolution);
        if (x >= 0 && x < static_cast<int>(grid.info.width) && y >= 0 && y < static_cast<int>(grid.info.height)) {
                int index = y * grid.info.width + x;
                // Set the cost only if the calculated cost is 100
                if (calculateCostBasedOnHeight(point.z) == 100) {
                    grid.data[index] = 100;
                }
            }
        }

        // removeIsolatedHighCostPoints(grid, grid.info.width, grid.info.height);

        // // Initialize the entire costmap to 0
        // for (size_t idx = 0; idx < grid.data.size(); idx++) {
        //     grid.data[idx] = 0;
        // }

        // Apply gradient costs to neighbors
        for (size_t idx = 0; idx < grid.data.size(); idx++) {
            if (grid.data[idx] == 100) {
                int x = idx % grid.info.width;
                int y = idx / grid.info.width;
                applyGradientCosts(x, y, grid, grid.info.width, grid.info.height);
            }
        }

        // Set unset spots to 0
        for (size_t idx = 0; idx < grid.data.size(); idx++) {
            if (grid.data[idx] == -1) {
                grid.data[idx] = 0;
            }
        }

        // height_publisher_->publish(height_data);
        grid_publisher_->publish(grid);

        // Save the occupancy grid as a .pgm file
        std::string pgm_file_path = "/home/aidan/ana_bot/src/ana/rtab_maps/occupancy_grid.pgm";
        std::ofstream pgm_file(pgm_file_path, std::ios::out | std::ios::binary);
        pgm_file << "P5\n" << grid.info.width << " " << grid.info.height << "\n255\n";
        for (unsigned int y = 0; y < grid.info.height; y++) {
            for (unsigned int x = 0; x < grid.info.width; x++) {
                int index = x + (grid.info.height - y - 1) * grid.info.width;
                int8_t value = grid.data[index];
                unsigned char val = (value == -1) ? 255 : static_cast<unsigned char>(std::max(0, std::min(254, static_cast<int>(value))));
                pgm_file.write(reinterpret_cast<char*>(&val), sizeof(val));
            }
        }
        pgm_file.close();

        // Save the corresponding .yaml file
        std::ofstream yaml_file("/home/aidan/ana_bot/src/ana/rtab_maps/occupancy_grid.yaml");
        yaml_file << "image: ./occupancy_grid.pgm\n";
        yaml_file << "resolution: " << grid.info.resolution << "\n";
        yaml_file << "origin: [" 
                  << grid.info.origin.position.x << ", " 
                  << grid.info.origin.position.y << ", " 
                  << grid.info.origin.position.z << "]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: 0.65\n";
        yaml_file << "free_thresh: 0.196\n";
        yaml_file.close();

        publisher_->publish(output);
        pcl::io::savePCDFileBinary("/home/aidan/ana_bot/src/ana/rtab_maps/mapped_point_cloud.pcd", *cloud); // Save the point cloud

        RCLCPP_INFO(this->get_logger(), "Published filtered point cloud and saved occupancy grid");

        timer_->cancel(); // Cancel timer
    }

    int calculateCostBasedOnHeight(float height) {
        float clearanceHeight = 0.2; // example clearance height
        if (height > 0.0 && height < clearanceHeight) {
            return 100;  // High cost
        }
        return 0;  // Default cost (if height <= 0.01)
    }

    void applyGradientCosts(int x, int y, nav_msgs::msg::OccupancyGrid& costmap, int width, int height) {
     
    }

    void removeIsolatedHighCostPoints(nav_msgs::msg::OccupancyGrid& costmap, int width, int height) {
        std::vector<signed char> new_data = costmap.data;  // Use the same data type as the original

        const int check_range = 6;  // Define the range around each cell to check for neighboring high-cost points
        const int required_neighbors = 1;  // Minimum number of high-cost neighbors

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                if (costmap.data[idx] == 100) {  // Check only high-cost cells
                    int neighbor_count = 0;

                    // Check surrounding cells
                    for (int ny = -check_range; ny <= check_range; ++ny) {
                        for (int nx = -check_range; nx <= check_range; ++nx) {
                            if (nx == 0 && ny == 0) continue;  // Skip the center cell
                            int nX = x + nx;
                            int nY = y + ny;
                            if (nX >= 0 && nX < width && nY >= 0 && nY < height) {
                                int nIdx = nY * width + nX;
                                if (costmap.data[nIdx] == 100) {
                                    neighbor_count++;
                                }
                            }
                        }
                    }

                    // If insufficient high-cost neighbors, reset the cost
                    if (neighbor_count < required_neighbors) {
                        new_data[idx] = 0;  // Reset to zero or another appropriate value
                    }
                }
            }
        }

        costmap.data = new_data;  // Update the costmap data with the modified values
    }




    nav_msgs::msg::OccupancyGrid createOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float resolution) {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = "map"; // Make sure this matches your point cloud frame_id
        grid.info.resolution = resolution;

        // Find cloud bounds
        float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();

        for (const auto& point : cloud->points) {
            if (point.x < min_x) min_x = point.x;
            if (point.x > max_x) max_x = point.x;
            if (point.y < min_y) min_y = point.y;
            if (point.y > max_y) max_y = point.y;
        }

        // Calculate grid dimensions
        grid.info.width = static_cast<uint32_t>((max_x - min_x) / resolution);
        grid.info.height = static_cast<uint32_t>((max_y - min_y) / resolution);
        grid.info.origin.position.x = min_x;
        grid.info.origin.position.y = min_y;
        grid.info.origin.position.z = 0;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(grid.info.width * grid.info.height, -1);  // Initialize all values to -1

        // Populate the occupancy grid
        for (const auto& point : cloud->points) {
            int x = static_cast<int>((point.x - min_x) / resolution);
            int y = static_cast<int>((point.y - min_y) / resolution);
            if (x >= 0 && x < static_cast<int>(grid.info.width) && y >= 0 && y < static_cast<int>(grid.info.height)) {
                grid.data[y * grid.info.width + x] = 100;  // Mark as occupied
            }
        }

        return grid;
    }


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr height_publisher_; 

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
