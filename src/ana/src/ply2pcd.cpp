#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

class PCDPublisher : public rclcpp::Node {
public:
    PCDPublisher() : Node("pcd_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_point_cloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10000),
            std::bind(&PCDPublisher::timerCallback, this));
    }

private:
    void timerCallback() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/aidan/ana_bot/src/ana/rtab_maps/orange_cones_3d.pcd", *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the file orange_cones_3d.pcd");
            return;
        }

        // RANSAC Plane Segmentation
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

            // Apply additional Voxel Grid to the plane to make it sparser
            pcl::VoxelGrid<pcl::PointXYZ> voxelFilterPlane;
            voxelFilterPlane.setInputCloud(plane_cloud);
            voxelFilterPlane.setLeafSize(0.02f, 0.02f, 0.02f); // Increase these values to make the plane less dense
            voxelFilterPlane.filter(*plane_cloud);

            // Combine the plane and non-plane points
            *non_plane_cloud += *plane_cloud;
            cloud.swap(non_plane_cloud); // Now cloud contains both sparsely sampled plane and other points
        }

        // // Apply Voxel Grid Downsampling
        // pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
        // voxelFilter.setInputCloud(cloud);
        // voxelFilter.setLeafSize(0.03f, 0.03f, 0.03f); // Increase these values to make the floor less dense
        // voxelFilter.filter(*cloud);

        // Apply PassThrough Filter to remove floor
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 5.0); // Only keep points that are between these heights.
        pass.filter(*cloud);

        // Apply Random Sampling
        pcl::RandomSample<pcl::PointXYZ> randomSample;
        randomSample.setInputCloud(cloud);
        randomSample.setSample(cloud->size() / 2); // Keep 50% of points, change to keep more/less.
        randomSample.filter(*cloud);

        // Apply Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50); // Number of neighbors to analyze for each point.
        sor.setStddevMulThresh(1.0); // Distance multiplier for determining which points are outliers.
        sor.filter(*cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = this->now();

        publisher_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published filtered point cloud");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
