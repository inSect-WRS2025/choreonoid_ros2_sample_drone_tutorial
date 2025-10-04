#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel")
    {
        // パラメータ（必要に応じて変更可）
        this->declare_parameter("axis_linear", 1);  // 左スティック上下（通常1）
        this->declare_parameter("axis_angular", 0); // 左スティック左右（通常0）
        this->declare_parameter("scale_linear", 0.5);
        this->declare_parameter("scale_angular", 1.0);

        get_parameter("axis_linear", axis_linear_);
        get_parameter("axis_angular", axis_angular_);
        get_parameter("scale_linear", scale_linear_);
        get_parameter("scale_angular", scale_angular_);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        if (msg->axes.size() > std::max(axis_linear_, axis_angular_)) {
            cmd.linear.x = msg->axes[axis_linear_] * scale_linear_;
            cmd.angular.z = msg->axes[axis_angular_] * scale_angular_;
            cmd_pub_->publish(cmd);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    int axis_linear_, axis_angular_;
    double scale_linear_, scale_angular_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
