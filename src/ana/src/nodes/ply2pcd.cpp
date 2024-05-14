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



// cuda kernels
extern "C" void processPointCloudVoxelGrid(PointXYZ *hostPoints, int numPoints, float voxelSize);

class PCDPublisher : public rclcpp::Node {
public:
    PCDPublisher() : Node("pcd_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_point_cloud", 10);
        grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy", 10);
        height_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("height_map", 10);  // Height publisher
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&PCDPublisher::timerCallback, this));
    }

private:
    void timerCallback() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

<<<<<<< HEAD
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/aidan/ana_bot/src/ana/rtab_maps/orange_cones_3d.pcd", *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the file orange_cones_3d.pcd");
=======
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/aidan/ana_bot/src/ana/rtab_maps/settings4.pcd", *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the pcd file");
>>>>>>> 72f5edd (SLAM, Costmap generation)
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
            voxelFilterPlane.setLeafSize(0.02f, 0.02f, 0.02f); // Increase these values to make the plane less dense
            voxelFilterPlane.filter(*plane_cloud);

            // Apply Voxel Grid Downsampling to non-planes - PCL GPU
            pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
            voxelFilter.setInputCloud(non_plane_cloud);
            voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f); // Increase these values to make the floor less dense
            voxelFilter.filter(*non_plane_cloud);
        
            // // Convert plane_cloud to CUDA and perform voxel grid downsampling
            // std::vector<PointXYZ> planePoints(plane_cloud->points.size());
            // for (size_t i = 0; i < plane_cloud->points.size(); ++i) {
            //     planePoints[i].x = plane_cloud->points[i].x;
            //     planePoints[i].y = plane_cloud->points[i].y;
            //     planePoints[i].z = plane_cloud->points[i].z;
            // }
            // processPointCloudVoxelGrid(planePoints.data(), planePoints.size(), 0.02f);
            // // Reconstruct the clouds from the processed points
            // plane_cloud->points.resize(planePoints.size());
            // for (size_t i = 0; i < planePoints.size(); ++i) {
            //     plane_cloud->points[i].x = planePoints[i].x;
            //     plane_cloud->points[i].y = planePoints[i].y;
            //     plane_cloud->points[i].z = planePoints[i].z;
            // }

            // // Convert non_plane_cloud to CUDA and perform voxel grid downsampling
            // std::vector<PointXYZ> nonPlanePoints(non_plane_cloud->points.size());
            // for (size_t i = 0; i < non_plane_cloud->points.size(); ++i) {
            //     nonPlanePoints[i].x = non_plane_cloud->points[i].x;
            //     nonPlanePoints[i].y = non_plane_cloud->points[i].y;
            //     nonPlanePoints[i].z = non_plane_cloud->points[i].z;
            // }
            // processPointCloudVoxelGrid(nonPlanePoints.data(), nonPlanePoints.size(), 0.01f);
            // non_plane_cloud->points.resize(nonPlanePoints.size());
            // // Reconstruct the clouds from the processed points
            // for (size_t i = 0; i < nonPlanePoints.size(); ++i) {
            //     non_plane_cloud->points[i].x = nonPlanePoints[i].x;
            //     non_plane_cloud->points[i].y = nonPlanePoints[i].y;
            //     non_plane_cloud->points[i].z = nonPlanePoints[i].z;
            // }
        }

        // Combine the plane and non-plane points
        *non_plane_cloud += *plane_cloud;
        cloud.swap(non_plane_cloud); // Now cloud contains both sparsely sampled plane and other points

        // Apply PassThrough Filter to remove floor - CUDA
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 5.0); // Only keep points that are between these heights.
        pass.filter(*cloud);

        // Apply Random Sampling - CUDA
        pcl::RandomSample<pcl::PointXYZ> randomSample;
        randomSample.setInputCloud(cloud);
        randomSample.setSample(cloud->size() / 2); // Keep 50% of points, change to keep more/less.
        randomSample.filter(*cloud);

        // Apply Statistical Outlier Removal - CUDA
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50); // Number of neighbors to analyze for each point.
        sor.setStddevMulThresh(1.0); // Distance multiplier for determining which points are outliers.
        sor.filter(*cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = this->now();

        // Convert cloud to an occupancy grid
        auto grid = createOccupancyGrid(cloud, 0.05f);  // Example: 5cm grid cell size
        grid_publisher_->publish(grid);

        // Publish height data
        std_msgs::msg::Float32MultiArray height_data;
        height_data.data.resize(grid.info.width * grid.info.height, -1.0); // Initialize with -1.0 (unknown height)
        for (const auto& point : non_plane_cloud->points) {
            int x = static_cast<int>((point.x - grid.info.origin.position.x) / grid.info.resolution);
            int y = static_cast<int>((point.y - grid.info.origin.position.y) / grid.info.resolution);
            if (x >= 0 && x < static_cast<int>(grid.info.width) && y >= 0 && y < static_cast<int>(grid.info.height)) {
                height_data.data[y * grid.info.width + x] = point.z;  // Store height data
            }
        }
        height_publisher_->publish(height_data);

        publisher_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published filtered point cloud");
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
