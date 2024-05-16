#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>  // Include for cv::Mat constants

// FILTER LIVE DEPTH IMAGE FROM D435i IN RTAB-MAP: ('depth/image', '/filtered/depth/image_raw')
// SUBSCRIBES /camera/depth/image_raw
// PUBLISHES /filtered/depth/image_raw

// cuda kernels
extern "C" void processDepthImage(float* hostImage, int width, int height, float maxDepth);

class DepthLiveFilterNode : public rclcpp::Node {
public:
    DepthLiveFilterNode() : Node("depth_live_filter_node") {
        // Constructor remains empty for safe use of shared_from_this
    }

    void initialize() {
        image_transport::ImageTransport it(shared_from_this());
        sub_ = it.subscribe("/camera/depth/image_raw", 10, &DepthLiveFilterNode::callback, this);
        pub_ = it.advertise("/filtered/depth/image_raw", 10);
    }

    // WORKING: preproceFss depth camera before RTAB-MAP uses it
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Process the depth image to reduce noise and range
        float max_valid_depth = 9.99;  // maximum depth in meters
        processDepthImage(reinterpret_cast<float*>(cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, max_valid_depth);

        // // Thresholding the depth to reduce the depth range, eliminating far points
        // float depth_limit = 5.0;  // Keep points within 5 meters
        // cv::threshold(cv_ptr->image, cv_ptr->image, depth_limit, 0, cv::THRESH_TOZERO_INV);

        // Optionally apply further downsampling
        cv::Mat reduced;
        int downsampling_factor = 1;  // Reduce the number of points by xN in each dimension
        cv::resize(cv_ptr->image, reduced, cv::Size(), 1.0 / downsampling_factor, 1.0 / downsampling_factor, cv::INTER_NEAREST);

        // Convert back to ROS message and publish
        cv_bridge::CvImage out_cv(cv_ptr->header, sensor_msgs::image_encodings::TYPE_32FC1, reduced);
        sensor_msgs::msg::Image::SharedPtr out_msg = out_cv.toImageMsg();
        pub_.publish(*out_msg);
    }


private:
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthLiveFilterNode>();
    node->initialize(); // Ensure `shared_from_this` is safely used
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
