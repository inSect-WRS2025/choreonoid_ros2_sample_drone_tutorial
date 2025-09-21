#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class JoyToPanTiltConverter : public rclcpp::Node
{
public:
    JoyToPanTiltConverter() : Node("joy_to_pan_tilt_converter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyToPanTiltConverter::joy_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("cmd_joint_vel", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto cmd_msg = geometry_msgs::msg::Vector3();
        
        // PS5のL2/R2ボタンは、通常はaxes[2]とaxes[5]に割り当てられます
        double l2_input = 0.0;
        if (msg->axes.size() > 2) {
            l2_input = (1.0 - msg->axes[2]) / 2.0; // L2: Pan
        }

        double r2_input = 0.0;
        if (msg->axes.size() > 5) {
            r2_input = (1.0 - msg->axes[5]) / 2.0; // R2: Tilt
        }
        
        // PanはZ軸、TiltはY軸に対応
        cmd_msg.z = l2_input;
        cmd_msg.y = r2_input;
        
        publisher_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToPanTiltConverter>());
    rclcpp::shutdown();
    return 0;
}