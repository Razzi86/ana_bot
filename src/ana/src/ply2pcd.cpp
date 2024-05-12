#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class PCDPublisher : public rclcpp::Node {
public:
    PCDPublisher() : Node("pcd_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_point_cloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PCDPublisher::timerCallback, this));
    }

private:
    void timerCallback() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/aidan/ana_bot/src/ana/rtab_maps/orange_cones_3d.pcd", *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the file orange_cones_3d.pcd");
            return;
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = this->now();

        publisher_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published point cloud");
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
