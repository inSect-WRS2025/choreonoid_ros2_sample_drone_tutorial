// JoyMultiplexer.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <array>
#include <vector>

class JoyMultiplexer : public rclcpp::Node {
public:
  JoyMultiplexer() : Node("joy_multiplexer_node"), mode(0) {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyMultiplexer::joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    flipper_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/flipper_command", 10);
    arm_left_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_command_left", 10);
    arm_right_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_command_right", 10);

    last_flipper_pos_.fill(0.0f);
    activeInput_.fill(false);

    RCLCPP_INFO(this->get_logger(), "Joy multiplexer started");
  }

private:
  enum Mode { CRAWLER = 0, FLIPPER = 1, ARM_LEFT = 2, ARM_RIGHT = 3 };
  int mode;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr flipper_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_right_pub_;

  // ヒステリシス用の状態保持（Joy側はオフセットを出す）
  std::array<float,4> last_flipper_pos_;
  std::array<bool,4> activeInput_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // モード切替
    if (msg->buttons.size() > 3 && msg->buttons[3]) mode = CRAWLER;       // □
    else if (msg->buttons.size() > 1 && msg->buttons[1]) mode = FLIPPER;  // ○
    else if (msg->buttons.size() > 0 && msg->buttons[0]) mode = ARM_LEFT; // ☓
    else if (msg->buttons.size() > 2 && msg->buttons[2]) mode = ARM_RIGHT;// △

    float lx = (msg->axes.size() > 0) ? msg->axes[0] : 0.0f; // 左スティック左右
    float ly = (msg->axes.size() > 1) ? msg->axes[1] : 0.0f; // 左スティック上下
    float rx = (msg->axes.size() > 3) ? msg->axes[3] : 0.0f; // 右スティック左右
    float ry = (msg->axes.size() > 4) ? msg->axes[4] : 0.0f; // 右スティック上下

    switch (mode) {
      case CRAWLER: {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = ly;
        twist.angular.z = lx;
        twist_pub_->publish(twist);
        break;
      }
      case FLIPPER: {
    	double deadzone = 0.05;
    	double max_offset = 1.2; // スティック最大で±1.2rad のオフセットを与える（要調整）

    	double ry = (msg->axes.size() > 4) ? msg->axes[4] : 0.0;

    	std::vector<double> offset = {0.0, 0.0, 0.0, 0.0};

    	if (std::fabs(ry) > deadzone) {
        	// 全フリッパ同時（ボタン未押下）
        	if (!msg->buttons[5] && !msg->buttons[4] && !msg->buttons[7] && !msg->buttons[6]) {
            		offset[0] =  max_offset * ry;   // left
            		offset[1] = -max_offset * ry;   // right (mirror)
            		offset[2] = -max_offset * ry;   // rear left
            		offset[3] =  max_offset * ry;   // rear right
        	} else {
            		if (msg->buttons[5]) offset[3] = -max_offset * ry; // R1 -> right rear
            		if (msg->buttons[4]) offset[2] =  max_offset * ry; // L1 -> left rear
            		if (msg->buttons[7]) offset[1] = -max_offset * ry; // R2 -> right front
            		if (msg->buttons[6]) offset[0] =  max_offset * ry; // L2 -> left front
        	}
    	}
    	sensor_msgs::msg::JointState js;
    	js.header.stamp = this->now();
    	js.name = {"FlipperLeftJoint","FlipperRightJoint","FlipperRearLeftJoint","FlipperRearRightJoint"};
    	js.position = offset; // **瞬時オフセット（中立ならゼロ）**
    	flipper_pub_->publish(js);
    	break;
	}

      case ARM_LEFT:
      case ARM_RIGHT: {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = {"Arm2", "Arm3", "Arm4", "Arm5", "ArmHand"};
        js.position.resize(5);
        js.position[0] = ly;
        js.position[1] = lx;
        js.position[2] = ry;
        js.position[3] = rx;
        js.position[4] = 0.0;

        if (mode == ARM_LEFT) arm_left_pub_->publish(js);
        else arm_right_pub_->publish(js);

        break;
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyMultiplexer>());
  rclcpp::shutdown();
  return 0;
}
