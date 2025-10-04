#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
 #include <algorithm>
 #include <array>

class JoyMultiplexer : public rclcpp::Node {
public:
  JoyMultiplexer() : Node("joy_multiplexer") {
    // Parameters
    axis_lx_ = this->declare_parameter<int>("axis_lx", 0);
    axis_ly_ = this->declare_parameter<int>("axis_ly", 1);
    axis_rx_ = this->declare_parameter<int>("axis_rx", 3);
    axis_ry_ = this->declare_parameter<int>("axis_ry", 4);
    flipper_modifier_button_ = this->declare_parameter<int>("flipper_modifier_button", 1); // ○
    flipper_absolute_mode_ = this->declare_parameter<bool>("flipper_absolute_mode", false);
    flipper_increment_front_ = this->declare_parameter<double>("flipper_increment_front", 0.02);
    flipper_increment_rear_ = this->declare_parameter<double>("flipper_increment_rear", 0.02);
    flipper_deadzone_ = this->declare_parameter<double>("flipper_deadzone", 0.2);
    flipper_rate_hz_ = this->declare_parameter<double>("flipper_rate_hz", 10.0);
    linear_scale_ = this->declare_parameter<double>("linear_scale", 1.0);
    angular_scale_ = this->declare_parameter<double>("angular_scale", 1.0);

    // pubs/subs
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyMultiplexer::joy_callback, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    flipper_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/flipper_command", 10);

    // timer for stepped flipper command
    using namespace std::chrono_literals;
    auto period_ms = static_cast<int>(1000.0 / std::max(1e-3, flipper_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&JoyMultiplexer::flipper_timer_tick, this));

    RCLCPP_INFO(this->get_logger(), "joy_multiplexer started (abs=%s, rate=%.2fHz)",
      flipper_absolute_mode_ ? "true" : "false", flipper_rate_hz_);
  }

private:
  // parameters
  int axis_lx_, axis_ly_, axis_rx_, axis_ry_;
  int flipper_modifier_button_;
  bool flipper_absolute_mode_;
  double flipper_increment_front_, flipper_increment_rear_;
  double flipper_deadzone_;
  double flipper_rate_hz_;
  double linear_scale_, angular_scale_;

  // state
  std::vector<float> last_axes_;
  std::vector<int32_t> last_buttons_;
  bool modifier_held_ = false;
  std::array<double,4> flipper_target_ {0.0, 0.0, 0.0, 0.0}; // abs mode accumulation

  // ros
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr flipper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr double FLIPPER_LIMIT = 1.57; // ±90deg

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Always drive base from left stick
    if ((int)msg->axes.size() > std::max(axis_lx_, axis_ly_)) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = linear_scale_ * msg->axes[axis_ly_];
      twist.angular.z = angular_scale_ * msg->axes[axis_lx_];
      twist_pub_->publish(twist);
    }

    // cache
    last_axes_ = msg->axes;
    last_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
    modifier_held_ = ((int)msg->buttons.size() > flipper_modifier_button_) && (msg->buttons[flipper_modifier_button_] != 0);
  }

  void flipper_timer_tick() {
    if (!modifier_held_) return; // only when modifier is held

    // Guard index access
    if ((int)last_axes_.size() <= std::max(axis_rx_, axis_ry_)) return;

    const double rx = last_axes_[axis_rx_];
    const double ry = last_axes_[axis_ry_];

    auto step_front = [&](double s){
      // front pair: left += s, right -= s
      flipper_target_[0] = std::clamp(flipper_target_[0] + s, -FLIPPER_LIMIT, FLIPPER_LIMIT);
      flipper_target_[1] = std::clamp(flipper_target_[1] - s, -FLIPPER_LIMIT, FLIPPER_LIMIT);
    };
    auto step_rear = [&](double s){
      // rear pair: left += s, right -= s
      flipper_target_[2] = std::clamp(flipper_target_[2] + s, -FLIPPER_LIMIT, FLIPPER_LIMIT);
      flipper_target_[3] = std::clamp(flipper_target_[3] - s, -FLIPPER_LIMIT, FLIPPER_LIMIT);
    };

    bool any = false;
    if (std::abs(ry) > flipper_deadzone_) {
      const double dir = (ry > 0.0) ? 1.0 : -1.0;
      if (flipper_absolute_mode_) step_front(dir * flipper_increment_front_);
      any = true;
    }
    if (std::abs(rx) > flipper_deadzone_) {
      const double dir = (rx > 0.0) ? 1.0 : -1.0;
      if (flipper_absolute_mode_) step_rear(dir * flipper_increment_rear_);
      any = true;
    }

    if (!any) return;

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {"FlipperLeftJoint", "FlipperRightJoint", "FlipperRearLeftJoint", "FlipperRearRightJoint"};

    if (flipper_absolute_mode_) {
      js.position = {flipper_target_[0], flipper_target_[1], flipper_target_[2], flipper_target_[3]};
    } else {
      double df_front = (std::abs(ry) > flipper_deadzone_) ? ((ry > 0.0) ? +flipper_increment_front_ : -flipper_increment_front_) : 0.0;
      double df_rear  = (std::abs(rx) > flipper_deadzone_) ? ((rx > 0.0) ? +flipper_increment_rear_  : -flipper_increment_rear_)  : 0.0;
      // offsets: [front L, front R, rear L, rear R]
      js.position = {df_front, -df_front, df_rear, -df_rear};
    }
    flipper_pub_->publish(js);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyMultiplexer>());
  rclcpp::shutdown();
  return 0;
}