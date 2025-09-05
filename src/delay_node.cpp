#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class DelayNode : public rclcpp::Node
{
public:
    DelayNode() : Node("delay_node")
    {
        // Publishers
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_delayed", 10);

        // Paramètre pour le délai initial (0s = Terre)
        this->declare_parameter<double>("delay_sec", 0.0);
        current_delay_ = this->get_parameter("delay_sec").as_double();

        // Subscribers
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_user", 10,
            std::bind(&DelayNode::onTwistReceived, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fleet_0/camera/image", 10,
            std::bind(&DelayNode::onImageReceived, this, std::placeholders::_1));

        // Callback pour changer le délai à la volée
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DelayNode::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "DelayNode ready, delay=%.2fs", current_delay_);
    }

private:
    void onTwistReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (current_delay_ <= 0.0)
        {
            // Pas de délai, publier immédiatement
            pub_cmd_->publish(*msg);
        }
        else
        {
            // Délai avec thread détaché
            std::thread([this, msg, delay = current_delay_]() {
                std::this_thread::sleep_for(std::chrono::duration<double>(delay));
                if (rclcpp::ok() && pub_cmd_) {
                    pub_cmd_->publish(*msg);
                }
            }).detach();
        }
    }

    void onImageReceived(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (current_delay_ <= 0.0)
        {
            // Pas de délai, publier immédiatement
            pub_image_->publish(*msg);
        }
        else
        {
            // Délai avec thread détaché
            std::thread([this, msg, delay = current_delay_]() {
                std::this_thread::sleep_for(std::chrono::duration<double>(delay));
                if (rclcpp::ok() && pub_image_) {
                    pub_image_->publish(*msg);
                }
            }).detach();
        }
    }

    rcl_interfaces::msg::SetParametersResult
    parameters_callback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params) {
            if (p.get_name() == "delay_sec") {
                current_delay_ = p.as_double();
                RCLCPP_INFO(get_logger(), "Delay updated to %.2fs (affects cmd_vel and camera)", current_delay_);
            }
        }
        
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        res.reason = "ok";
        return res;
    }

    // Membres
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    
    double current_delay_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
