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
    // --- パラメータ宣言 ---
    this->declare_parameter<int>("crawler_button", 3); // □
    this->declare_parameter<int>("flipper_button", 1); // ○
    this->declare_parameter<int>("arm_left_button", 0); // ☓
    this->declare_parameter<int>("arm_right_button", 2); // △

    this->declare_parameter<int>("axis_lx", 0);
    this->declare_parameter<int>("axis_ly", 1);
    this->declare_parameter<int>("axis_rx", 3);
    this->declare_parameter<int>("axis_ry", 4);

    // 値読み込み
    load_params();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyMultiplexer::joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    flipper_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/flipper_command", 10);
    arm_left_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_command_left", 10);
    arm_right_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_command_right", 10);

    RCLCPP_INFO(this->get_logger(), "Joy multiplexer started with config params");
  }

private:
  enum Mode { CRAWLER = 0, FLIPPER = 1, ARM_LEFT = 2, ARM_RIGHT = 3 };
  int mode;

  // 設定値（ボタン・軸番号）
  int crawler_button_, flipper_button_, arm_left_button_, arm_right_button_;
  int axis_lx_, axis_ly_, axis_rx_, axis_ry_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr flipper_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_right_pub_;

  void load_params() {
    crawler_button_ = this->get_parameter("crawler_button").as_int();
    flipper_button_ = this->get_parameter("flipper_button").as_int();
    arm_left_button_ = this->get_parameter("arm_left_button").as_int();
    arm_right_button_ = this->get_parameter("arm_right_button").as_int();

    axis_lx_ = this->get_parameter("axis_lx").as_int();
    axis_ly_ = this->get_parameter("axis_ly").as_int();
    axis_rx_ = this->get_parameter("axis_rx").as_int();
    axis_ry_ = this->get_parameter("axis_ry").as_int();
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // モード切替
    if (msg->buttons.size() > crawler_button_ && msg->buttons[crawler_button_]) mode = CRAWLER;
    else if (msg->buttons.size() > flipper_button_ && msg->buttons[flipper_button_]) mode = FLIPPER;
    else if (msg->buttons.size() > arm_left_button_ && msg->buttons[arm_left_button_]) mode = ARM_LEFT;
    else if (msg->buttons.size() > arm_right_button_ && msg->buttons[arm_right_button_]) mode = ARM_RIGHT;

    // 軸値取得
    float lx = (msg->axes.size() > axis_lx_) ? msg->axes[axis_lx_] : 0.0f;
    float ly = (msg->axes.size() > axis_ly_) ? msg->axes[axis_ly_] : 0.0f;
    float rx = (msg->axes.size() > axis_rx_) ? msg->axes[axis_rx_] : 0.0f;
    float ry = (msg->axes.size() > axis_ry_) ? msg->axes[axis_ry_] : 0.0f;

    switch (mode) {
      case CRAWLER: {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = -ly;
        twist.angular.z = lx;
        twist_pub_->publish(twist);
        break;
      }
      case FLIPPER: {
        double deadzone = 0.05;
        double max_offset = 1.2;

        std::vector<double> offset = {0.0, 0.0, 0.0, 0.0};
        
        // R1 + 右スティック上下 -> 右後フリッパ昇降
        if (msg->buttons.size() > 5 && msg->buttons[5]) {
          offset[3] = -max_offset * ry;
        }
        // L1 + 右スティック上下 -> 左後フリッパ昇降
        if (msg->buttons.size() > 4 && msg->buttons[4]) {
          offset[2] = max_offset * ry;
        }
        // R2 + 右スティック上下 -> 右前フリッパ昇降
        if (msg->buttons.size() > 7 && msg->buttons[7]) {
          offset[1] = -max_offset * ry;
        }
        // L2 + 右スティック上下 -> 左前フリッパ昇降
        if (msg->buttons.size() > 6 && msg->buttons[6]) {
          offset[0] = max_offset * ry;
        }

        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = {"FlipperLeftJoint","FlipperRightJoint","FlipperRearLeftJoint","FlipperRearRightJoint"};
        js.position = offset;
        flipper_pub_->publish(js);
        break;
      }
      case ARM_LEFT:
      case ARM_RIGHT: {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();

        // ジョイント名を正確に設定
        js.name = {"ArmLeft1Joint", "ArmLeft2Joint", "ArmLeft3Joint", "ArmLeft4Joint", "ArmLeft5Joint", "ArmLeftHandJoint"};
        
        // 右腕モードの場合はジョイント名を右腕用に変更
        if (mode == ARM_RIGHT) {
            js.name = {"ArmRight1Joint", "ArmRight2Joint", "ArmRight3Joint", "ArmRight4Joint", "ArmRight5Joint", "ArmRightHandJoint"};
        }

        js.position.resize(js.name.size());
        
        // ジョイント軸とコントローラの軸をマッピング
        // PS5コントローラーのボタンと軸のデフォルトIDを使用
        
        // ジョイント第一関節 (左スティック左右)
        js.position[0] = lx;
        // ジョイント第二関節 (左スティック上下)
        js.position[1] = ly;
        // ジョイント第三関節 (右スティック左右)
        js.position[2] = rx;
        // ジョイント第四関節 (右スティック上下)
        js.position[3] = ry;
        // ジョイント第五関節 (未割り当て)
        js.position[4] = 0.0;
        // ジョイント第六関節 (ハンド) (未割り当て)
        js.position[5] = 0.0;

        // publish
        if (mode == ARM_LEFT) {
            arm_left_pub_->publish(js);
        } else {
            arm_right_pub_->publish(js);
        }
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

