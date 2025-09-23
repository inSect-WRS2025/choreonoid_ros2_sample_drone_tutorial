#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>

class MobileRobotPanTiltController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    cnoid::Link* joints[2]; // [0]=Pan, [1]=Tilt
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist command;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor;
    std::thread executorThread;
    std::mutex commandMutex;

    double target_q[2];   // 目標角度
    double dt;            // 制御周期
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotPanTiltController)

bool MobileRobotPanTiltController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    // /angler を購読
    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "/angler", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg){
            std::lock_guard<std::mutex> lock(commandMutex);
            command = *msg;
        });
        
    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    return true;
}

bool MobileRobotPanTiltController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    joints[0] = body->joint("PanJoint");
    joints[1] = body->joint("TiltJoint");

    // 位置制御モード
    for(int i = 0; i < 2; ++i){
        cnoid::Link* joint = joints[i];
        joint->setActuationMode(cnoid::Link::JointDisplacement);
        io->enableOutput(joint, cnoid::Link::JointDisplacement);
        target_q[i] = joint->q(); // 初期角度
    }

    dt = io->timeStep();
    return true;
}

bool MobileRobotPanTiltController::control()
{
    double dq_target[2];

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        dq_target[0] = command.angular.x; // Pan ← angler.x (rad/s)
        dq_target[1] = command.angular.y; // Tilt ← angler.y (rad/s)
    }
    
    // 速度を積分して角度に変換
    for(int i = 0; i < 2; ++i){
        target_q[i] += dq_target[i] * dt;

        // 可動域制限（-360°～+360°）
        const double limit = 2.0 * M_PI;
        if(target_q[i] >  limit) target_q[i] -= 2.0 * M_PI;
        if(target_q[i] < -limit) target_q[i] += 2.0 * M_PI;

        joints[i]->q_target() = target_q[i];
    }

    return true;
}

void MobileRobotPanTiltController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}
