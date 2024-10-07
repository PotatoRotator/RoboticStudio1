#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/empty.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
 
class LaserScan : public rclcpp::Node
{
public:
    LaserScan() : Node("laser_scan")
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));
        angle_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/new_scan", 10);
        n_ = 1;
        count = 0;
    }
private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if(count == 0) {
            std::cout << "Enter a value for n: " << std::endl;
            std::cin >> n_;
            count = 1;
        }

        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

        std::vector<float> new_ranges(msg->ranges.size(), std::numeric_limits<float>::infinity());
        std::vector<float> new_intensities(msg->intensities.size(), std::numeric_limits<float>::quiet_NaN());

        for (size_t i = 0; i < msg->ranges.size(); i += n_)
        {
             new_ranges[i] = msg->ranges[i];
            if (!msg->intensities.empty()) {
                new_intensities[i] = msg->intensities[i];
            }
        }

        new_scan->ranges = new_ranges;
        if (!msg->intensities.empty()) {
            new_scan->intensities = new_intensities;
        }

        RCLCPP_INFO(this->get_logger(), "Processed scan data:");
        for (size_t i = 0; i < new_ranges.size(); ++i)
        {
            if (i % n_ == 0) {
                RCLCPP_INFO(this->get_logger(), "[Angle Index %zu]: Range = %f, Intensity = %f", 
                            i, new_ranges[i], new_intensities[i]);
            }
        }

        // Publish the modified scan
        angle_pub->publish(*new_scan);
        
        
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr angle_pub;
    int n_;
    int count;
};
 
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return 0;
}