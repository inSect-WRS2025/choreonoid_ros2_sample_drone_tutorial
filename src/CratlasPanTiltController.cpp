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

class CratlasPanTiltController : public cnoid::SimpleController
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
    
    // 十字ボタンで向かわせる目標角 (ここでパラメータ調節してください)
    const double up_pan_q = 0.0;         // 十字ボタン上: Pan目標角
    const double up_tilt_q = 0.25;       // 十字ボタン上: Tilt目標角 (例: 上を向く)

    const double down_pan_q = -3.14;       // 十字ボタン下: Pan目標角
    const double down_tilt_q = 0.6;      // 十字ボタン下: Tilt目標角 (例: 下を向く)

    const double left_pan_q = 1.57;       // 十字ボタン左: Pan目標角 (例: 左を向く)
    const double left_tilt_q = 0.45;      // 十字ボタン左: Tilt目標角

    const double right_pan_q = -1.57;     // 十字ボタン右: Pan目標角 (例: 右を向く)
    const double right_tilt_q = 0.45;     // 十字ボタン右: Tilt目標角
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CratlasPanTiltController)

bool CratlasPanTiltController::configure(cnoid::SimpleControllerConfig* config)
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

bool CratlasPanTiltController::initialize(cnoid::SimpleControllerIO* io)
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

bool CratlasPanTiltController::control()
{
    bool button_pressed = false;

    // PS5ボタン処理
    if(joystick->isReady()){
        joystick->readCurrentState(); // ジョイスティックの状態を更新

        // 十字ボタンの入力を取得 (古いバージョンのChoreonoid向け)
        double hat_x = joystick->getPosition(4); // 通常はインデックス5が十字ボタンの左右
        double hat_y = joystick->getPosition(5); // 通常はインデックス6が十字ボタンの上下

        if (hat_x > 0.5) { // 十字ボタン右
            target_q[0] = right_pan_q;
            target_q[1] = right_tilt_q;
            button_pressed = true;
        } else if (hat_x < -0.5) { // 十字ボタン左
            target_q[0] = left_pan_q;
            target_q[1] = left_tilt_q;
            button_pressed = true;
        } else if (hat_y > 0.5) { // 十字ボタン下
            target_q[0] = down_pan_q;
            target_q[1] = down_tilt_q;
            button_pressed = true;
        } else if (hat_y < -0.5) { // 十字ボタン上
            target_q[0] = up_pan_q;
            target_q[1] = up_tilt_q;
            button_pressed = true;
        }
    }
    
    if(!button_pressed){
        // どちらのボタンも押されていない場合、ROSからのTwistコマンドを使用
        std::lock_guard<std::mutex> lock(commandMutex);
        target_q[0] += command.angular.x * dt; // Pan ← angler.x (rad/s)
        target_q[1] += command.angular.y * dt; // Tilt ← angler.y (rad/s)
    }

    // 可動域制限（-360°～+360°）
    for(int i = 0; i < 2; ++i){
        const double limit = 2.0 * M_PI;
        if(target_q[i] >  limit) target_q[i] -= 2.0 * M_PI;
        if(target_q[i] < -limit) target_q[i] += 2.0 * M_PI;

        joints[i]->q_target() = target_q[i];
    }

    return true;
}

void CratlasPanTiltController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}