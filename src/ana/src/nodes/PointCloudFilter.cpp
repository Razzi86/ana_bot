#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>  // Include for cv::Mat constants

// cuda kernels
extern "C" void processDepthImage(float* hostImage, int width, int height, float maxDepth);

class PointCloudFilter : public rclcpp::Node {
public:
    PointCloudFilter() : Node("point_cloud_filter_node") {
        // Constructor remains empty for safe use of shared_from_this
    }

    void initialize() {
        image_transport::ImageTransport it(shared_from_this());
        sub_ = it.subscribe("/camera/depth/image_raw", 10, &PointCloudFilter::callback, this);
        pub_ = it.advertise("/filtered/depth/image_raw", 10);
    }

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Assuming depth values are in meters and converted to millimeters
        float max_valid_depth = 9.99;  // 9 meters in millimeters
        processDepthImage(reinterpret_cast<float*>(cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, max_valid_depth);

        sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
        pub_.publish(*out_msg);
    }

private:
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();
    node->initialize(); // Ensure `shared_from_this` is safely used
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
