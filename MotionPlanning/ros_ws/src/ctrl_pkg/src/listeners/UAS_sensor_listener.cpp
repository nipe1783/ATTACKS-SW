#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class UasSensorListener : public rclcpp::Node
{
public:
    UasSensorListener() : Node("uas_sensor_listener"), image_count_(0)
    {
        // Create a subscription to the camera topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&UasSensorListener::image_callback, this, std::placeholders::_1));
        
        // Set up a timer to save images once per second
        timer_ = this->create_wall_timer(1000ms, std::bind(&UasSensorListener::save_image, this));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image message to an OpenCV image
        try {
            last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void save_image()
    {
        if (!last_image_.empty()) {
            std::string filename = "saved_image_" + std::to_string(image_count_++) + ".jpg";
            cv::imwrite(filename, last_image_);
            RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat last_image_;
    size_t image_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UasSensorListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
