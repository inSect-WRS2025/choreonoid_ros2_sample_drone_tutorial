#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>

class AgxTankRos2Controller : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    cnoid::Link* sprockets[2];
    double track_width;
    double main_sprocket_radius_ = 0.10;    // [m]
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist command;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor;
    std::thread executorThread;
    std::mutex commandMutex;
};

bool AgxTankRos2Controller::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg){
            std::lock_guard<std::mutex> lock(commandMutex);
            command = *msg;
        });

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    return true;
}

bool AgxTankRos2Controller::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    sprockets[0] = body->joint("WHEEL_L0"); // Left Sprocket
    sprockets[1] = body->joint("WHEEL_R0"); // Right Sprocket

    if (!sprockets[0] || !sprockets[1]) {
        RCLCPP_ERROR(node->get_logger(), "Sprocket joints not found.");
        return false;
    }

    for(int i=0; i < 2; ++i){
        auto sprocket = sprockets[i];
        sprocket->setActuationMode(cnoid::Link::JointVelocity);
        io->enableOutput(sprocket);
    }
    
    // Calculate track width from joint positions
    track_width = std::abs(sprockets[0]->translation().y() - sprockets[1]->translation().y());
    RCLCPP_INFO(node->get_logger(), "Track width calculated: %f", track_width);

    return true;
}

bool AgxTankRos2Controller::control()
{
    double linear_vel = 0.0;
    double angular_vel = 0.0;

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        linear_vel = command.linear.x;
        angular_vel = command.angular.z;
    }

    // Differential drive kinematics (track linear speeds)
    double v_left = linear_vel - (angular_vel * track_width / 2.0);   // [m/s]
    double v_right = linear_vel + (angular_vel * track_width / 2.0);  // [m/s]
    // Limit linear speeds to keep physics stable
    const double v_max = 1.0; // [m/s]
    v_left = std::clamp(v_left, -v_max, v_max);
    v_right = std::clamp(v_right, -v_max, v_max);

    // Convert to wheel angular velocity [rad/s]
    const double w_left_main = v_left / std::max(1e-6, main_sprocket_radius_);
    const double w_right_main = v_right / std::max(1e-6, main_sprocket_radius_);
    const double w_max = 20.0; // [rad/s]
    sprockets[0]->dq_target() = std::clamp(w_left_main, -w_max, w_max);
    sprockets[1]->dq_target() = std::clamp(w_right_main, -w_max, w_max);

    return true;
}

void AgxTankRos2Controller::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AgxTankRos2Controller)