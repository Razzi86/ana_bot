#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

// CUDA kernel function declaration
extern "C" void processDepthImage(float* hostImage, int width, int height, float maxDepth);

class PointNetFilter : public rclcpp::Node {
public:
    PointNetFilter() : Node("point_net_filter") {
        // Use image_transport for subscribing to image topics
        auto transport = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        depth_subscriber_ = transport->subscribe("depth_image", 1, &PointNetFilter::depthCallback, this);
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            processDepthImage(reinterpret_cast<float*>(cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, 5.0f);  // Assuming max depth of 5 meters
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

private:
    image_transport::Subscriber depth_subscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointNetFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
