/**
   Reset Zero Velocity Controller
   - Subscribes `/drone/zero_vel` (std_msgs/Empty)
   - When triggered, zeroes root linear and angular velocity in place
     and zeroes all joint dq and u, then notifies kinematic change.
*/

#include <cnoid/Body>
#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <mutex>
#include <thread>
#include <memory>
#include <string>

class ResetZeroVelocityController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool start() override;
    virtual bool control() override;
    virtual void stop() override;
    virtual void unconfigure() override;

private:
    cnoid::SimpleControllerIO* io_ = nullptr;
    cnoid::BodyPtr body_;
    cnoid::Link* root_ = nullptr;

    // ROS 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor_;
    std::thread executor_thread_;

    // Trigger flag
    bool zero_req_ = false;
    std::mutex mtx_;

    std::string controller_name_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ResetZeroVelocityController)

bool ResetZeroVelocityController::configure(cnoid::SimpleControllerConfig* config)
{
    controller_name_ = config->controllerName();
    return true;
}

bool ResetZeroVelocityController::initialize(cnoid::SimpleControllerIO* io)
{
    io_ = io;
    body_ = io_->body();
    root_ = body_->rootLink();
    return true;
}

bool ResetZeroVelocityController::start()
{
    node_ = std::make_shared<rclcpp::Node>(controller_name_.empty() ? "drone_zero_vel_controller" : controller_name_);

    sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/drone/zero_vel", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            std::lock_guard<std::mutex> lock(mtx_);
            zero_req_ = true;
        });

    executor_ = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    return true;
}

bool ResetZeroVelocityController::control()
{
    bool do_zero = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if(zero_req_) {
            do_zero = true;
            zero_req_ = false;
        }
    }

    if(do_zero) {
        // Zero root linear and angular velocities at current pose
        root_->v().setZero();
        root_->w().setZero();

        // Also zero all joint velocities and inputs
        for(int i = 0; i < body_->numJoints(); ++i) {
            auto* j = body_->joint(i);
            j->dq() = 0.0;
            j->u()  = 0.0;
        }

        // Kinematic change will be reflected by the simulator on next step
    }

    return true;
}

void ResetZeroVelocityController::stop()
{
    if(executor_) {
        executor_->cancel();
        if(executor_thread_.joinable()) {
            executor_thread_.join();
        }
        executor_->remove_node(node_);
        executor_.reset();
    }
}

void ResetZeroVelocityController::unconfigure()
{
}
