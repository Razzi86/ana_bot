#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <cmath>

class GridSubscriber : public rclcpp::Node {
public:
    GridSubscriber() : Node("grid_subscriber") {
        occupancy_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy", 10, std::bind(&GridSubscriber::occupancy_callback, this, std::placeholders::_1));
        height_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "height_map", 10, std::bind(&GridSubscriber::height_callback, this, std::placeholders::_1));
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    }

private:
    void occupancy_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_occupancy_ = *msg;
        if (!height_data_.data.empty() && height_data_.data.size() == current_occupancy_.info.width * current_occupancy_.info.height) {
            generate_costmap();
        }
    }

    void height_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        height_data_ = *msg;
        if (current_occupancy_.data.size() > 0 && height_data_.data.size() == current_occupancy_.info.width * current_occupancy_.info.height) {
            generate_costmap();
        }
    }

    void generate_costmap() {
        auto costmap = current_occupancy_;
        int width = current_occupancy_.info.width;
        int height = current_occupancy_.info.height;

        // First, apply high cost for high heights
        for (size_t i = 0; i < current_occupancy_.data.size(); i++) {
            costmap.data[i] = calculateCostBasedOnHeight(height_data_.data[i]);
        }

        // Apply gradient costs to neighbors
        for (size_t idx = 0; idx < current_occupancy_.data.size(); idx++) {
            if (costmap.data[idx] == 100) {
                int x = idx % width;
                int y = idx / width;
                applyGradientCosts(x, y, costmap, width, height);
            }
        }

        costmap_publisher_->publish(costmap);
    }

    int calculateCostBasedOnHeight(float height) {

        // float robotHeight = 0.119;
        float clearanceHeight = 0.2;

        if (height > 0.03 && height < clearanceHeight) 
        {
            return 100;   // High cost
        }
        return 0;  // Default cost (if height <= 0.01)
    }


    void applyGradientCosts(int x, int y, nav_msgs::msg::OccupancyGrid& costmap, int width, int height) {
        const int range = 10;  // Consider only the first three cells around high-cost cells
        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                if (dx == 0 && dy == 0) continue;  // Skip the central cell
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
                    int nIdx = ny * width + nx;
                    float distance = std::sqrt(dx * dx + dy * dy);
                    int decayedCost = 0;
                    if (distance <= 7) decayedCost = 66;
                    else if (distance <= 9) decayedCost = 33;
                    else if (distance <= 11) decayedCost = 12;

                    if (costmap.data[nIdx] < decayedCost) {
                        costmap.data[nIdx] = decayedCost;
                    }
                }
            }
        }
    }

    // Member variables to store current state
    nav_msgs::msg::OccupancyGrid current_occupancy_;
    std_msgs::msg::Float32MultiArray height_data_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr height_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
