/**
   Twist + PanTilt Publisher Controller
   PS5 Controller support
*/

#include <cnoid/Joystick>
#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>

class TwistPublisherController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    enum ControlMode { DroneMode, PanTiltMode };

    cnoid::Joystick joystick;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angular_pub;
    rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
    std::thread executorThread;
    std::mutex commandMutex;

    ControlMode currentMode;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TwistPublisherController)

bool TwistPublisherController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    twist_pub  = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    angular_pub = node->create_publisher<geometry_msgs::msg::Twist>("/angler", 10);

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    return true;
}

bool TwistPublisherController::initialize(cnoid::SimpleControllerIO* io)
{
    currentMode = DroneMode;  // 初期状態はドローン制御
    return true;
}

bool TwistPublisherController::control()
{
    joystick.readCurrentState();

    // PS5 コントローラのボタン番号例
    // □=0, ○=1, ×=2, △=3
    if(joystick.getButtonState(3)) {   // △ボタン → PanTilt モード
        currentMode = PanTiltMode;
    }
    if(joystick.getButtonState(1)) {   // ○ボタン → Drone モード
        currentMode = DroneMode;
    }

    double pos[4];
    for(int i = 0; i < 4; ++i) {
        pos[i] = joystick.getPosition(i);
        if(fabs(pos[i]) < 0.2) {
            pos[i] = 0.0;
        }
    }

    {
        std::lock_guard<std::mutex> lock(commandMutex);

        if(currentMode == DroneMode) {
            // ドローン制御用 /cmd_vel
            static double vel[] = { 2.0, 2.0, 2.0, 1.047 };
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x  = vel[2] * pos[1] * -1.0;
            msg.linear.y  = vel[1] * pos[0] * -1.0;
            msg.linear.z  = vel[0] * pos[3] * -1.0;
            msg.angular.z = vel[3] * pos[2] * -1.0;
            twist_pub->publish(msg);

        } else if(currentMode == PanTiltMode) {
            // PanTilt 制御用 /angler
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.x = joystick.getPosition(6);
            msg.angular.y = joystick.getPosition(7);
            msg.angular.z = 0.0;
            angular_pub->publish(msg);
        }
    }

    return true;
}

void TwistPublisherController::unconfigure()
{
    if(executor) {
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}
