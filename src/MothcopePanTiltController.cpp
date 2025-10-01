#include <cnoid/SimpleController>
#include <cnoid/Joystick> // Joystickクラスを追加
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <cmath> // M_PI のために追加

// M_PIの定義がない環境のために
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    
    // PS5コントローラ関連の追加
    std::unique_ptr<cnoid::Joystick> joystick;
    const double initial_pan_q = 0.0;    // □ボタン: Pan初期目標角 (0 rad)
    const double initial_tilt_q = 0.0;   // □ボタン: Tilt初期目標角 (0 rad)
    
    // ❌ボタンで向かわせる目標角 (ここでパラメータ調節してください)
    const double cross_pan_q = 0.0;      // ❌ボタン: Pan目標角 (例: 0.5 rad)
    const double cross_tilt_q = 1.0;     // ❌ボタン: Tilt目標角 (例: 0.0 rad)
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotPanTiltController)

bool MobileRobotPanTiltController::configure(cnoid::SimpleControllerConfig* config)
{
    // Joystickの初期化
    joystick = std::make_unique<cnoid::Joystick>();
    if (!joystick->isReady()) {
        config->os() << "Joystick not found or initialization failed. ROS input only." << std::endl;
    }

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
    bool button_pressed = false;

    // PS5ボタン処理
    if(joystick->isReady()){
        joystick->readCurrentState(); // ジョイスティックの状態を更新

    // 修正後: getButtonState(ID) を使用
    if (joystick->getButtonState(cnoid::Joystick::X_BUTTON)) { // □ボタン (Square)
        target_q[0] = initial_pan_q;
        target_q[1] = initial_tilt_q;
        button_pressed = true;
    }
    else if (joystick->getButtonState(cnoid::Joystick::A_BUTTON)) { // ❌ボタン (Cross)
        target_q[0] = cross_pan_q;
        target_q[1] = cross_tilt_q;
        button_pressed = true;
    }
    }
    
    if(button_pressed){
        // ボタンが押されている場合は、Twist入力を無視し、積分も行わない
        dq_target[0] = 0.0; 
        dq_target[1] = 0.0;
    } else {
        // どちらのボタンも押されていない場合、ROSからのTwistコマンドを使用
        std::lock_guard<std::mutex> lock(commandMutex);
        dq_target[0] = command.angular.x; // Pan ← angler.x (rad/s)
        dq_target[1] = command.angular.y; // Tilt ← angler.y (rad/s)
    }

    // 速度を積分して角度に変換 (ボタンが押されていない場合のみ)
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