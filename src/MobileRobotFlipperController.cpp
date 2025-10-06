// MobileRobotFlipperController.cpp
#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std;
using namespace cnoid;

namespace {

const double STICK_THRESH = 0.1;

}

class MobileRobotFlipperController : public SimpleController
{
public:
    virtual bool configure(SimpleControllerConfig* config) override;
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    Link* flippers_[4]{nullptr, nullptr, nullptr, nullptr};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::array<double,4> cmd_{0.0,0.0,0.0,0.0};        // current target
    std::array<double,4> hold_pos_{0.0,0.0,0.0,0.0};  // hold base position
    std::mutex cmdMutex_;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor_;
    std::thread executorThread_;

    // 入力中フラグ（ヒステリシス）
    std::array<bool,4> activeInput_{false,false,false,false};

    // rate limit / prev values / moving average for dq
    std::array<double,4> prev_cmd_{0.0,0.0,0.0,0.0};
    std::array<double,4> prev_q_{0.0,0.0,0.0,0.0};
    std::array<double,4> ma_abs_dq_{0.0,0.0,0.0,0.0};

    // stall 判定
    std::array<int,4> stall_count_{0,0,0,0};
    std::array<double,4> last_warn_time_{0.0,0.0,0.0,0.0};
    std::array<bool,4> stall_locked_{false,false,false,false};

    const std::unordered_map<std::string, int> jointNameMap = {
        {"FlipperLeftJoint", 0},
        {"FlipperRightJoint", 1},
        {"FlipperRearLeftJoint", 2},
        {"FlipperRearRightJoint", 3}
    };
    
    // フリッパーごとの可動範囲を定義
    std::array<double, 4> flipper_min_limits_;
    std::array<double, 4> flipper_max_limits_;

    std::chrono::steady_clock::time_point lastLogTime_;
    
    // AizuSpiderControllerのフリッパー制御ロジックのための変数
    enum { FR_FLIPPER, FL_FLIPPER, BR_FLIPPER, BL_FLIPPER, NUM_FLIPPERS };
    SharedJoystickPtr joystick;
    int targetMode;

    // メンバ関数の宣言を追加
    void updateFlipperTargetPositions();
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotFlipperController)

bool MobileRobotFlipperController::configure(SimpleControllerConfig* config)
{
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }
    node_ = std::make_shared<rclcpp::Node>(config->controllerName());

    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "flipper_command", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg){
            std::lock_guard<std::mutex> lock(cmdMutex_);
            
            constexpr double deadband_enter = 0.02;

            for (size_t i = 0; i < msg->name.size(); ++i) {
                auto it = jointNameMap.find(msg->name[i]);
                if (it == jointNameMap.end()) continue;
                int idx = it->second;
                if (i >= msg->position.size()) continue;
                double offset = msg->position[i];

                if (std::abs(offset) > deadband_enter) {
                    activeInput_[idx] = true;
                    cmd_[idx] = hold_pos_[idx] + offset;
                    cmd_[idx] = std::clamp(cmd_[idx], flipper_min_limits_[idx], flipper_max_limits_[idx]);
                } else {
                    if (activeInput_[idx]) {
                        activeInput_[idx] = false;
                        hold_pos_[idx] = cmd_[idx];
                    }
                }
            }
        }
    );

    executor_ = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor_->add_node(node_);
    executorThread_ = std::thread([this](){ executor_->spin(); });

    lastLogTime_ = std::chrono::steady_clock::now();
    return true;
}

bool MobileRobotFlipperController::initialize(SimpleControllerIO* io)
{
    constexpr double DEG10_TO_RAD = 10.0 * M_PI / 180.0;
    
    const double DEG40_TO_RAD = 40.0 * M_PI / 180.0;
    const double DEG90_TO_RAD = 90.0 * M_PI / 180.0;
    
    flipper_min_limits_ = {
        -DEG90_TO_RAD,
        -DEG90_TO_RAD,
        -DEG40_TO_RAD,
        -DEG40_TO_RAD
    };
    flipper_max_limits_ = {
        DEG40_TO_RAD,
        DEG40_TO_RAD,
        DEG90_TO_RAD,
        DEG90_TO_RAD
    };

    auto body = io->body();
    for (int i = 0; i < 4; ++i) {
        flippers_[i] = nullptr;
    }

    flippers_[0] = body->joint(14); // FLIPPER_L_BASE
    flippers_[1] = body->joint(15); // FLIPPER_R_BASE
    flippers_[2] = body->joint(16); // FLIPPER_L_REAR_BASE
    flippers_[3] = body->joint(17); // FLIPPER_R_REAR_BASE

    const std::array<double, 4> initial_targets = {
        -DEG10_TO_RAD,
        DEG10_TO_RAD,
        -DEG10_TO_RAD,
        DEG10_TO_RAD
    };

    for (int i = 0; i < 4; ++i) {
        if (!flippers_[i]) {
            RCLCPP_WARN(node_->get_logger(), "Joint for flipper index %d not found.", i);
            continue;
        }
        flippers_[i]->setActuationMode(Link::JointTorque);
        io->enableInput(flippers_[i], Link::JointVelocity);
        io->enableOutput(flippers_[i], Link::JointTorque);

        hold_pos_[i] = initial_targets[i]; 
        cmd_[i] = initial_targets[i];      

        prev_cmd_[i] = cmd_[i];
        prev_q_[i] = flippers_[i]->q();
        ma_abs_dq_[i] = 0.0;
        stall_count_[i] = 0;
        stall_locked_[i] = false;

        RCLCPP_INFO(node_->get_logger(), "[Init][%d] joint name: %s, jointId: %d, hold_q=%.4f, init_target=%.4f", 
            i, flippers_[i]->name().c_str(), flippers_[i]->jointId(), hold_pos_[i], cmd_[i]);
    }
    
    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    targetMode = joystick->addMode();

    return true;
}

bool MobileRobotFlipperController::control()
{
    constexpr double kp = 7.0;
    constexpr double kd = 4.0;
    constexpr double error_deadband = 0.005;
    constexpr double dq_deadband = 0.02;

    constexpr double torque_stall_ratio = 0.8;
    constexpr int stall_threshold_count = 20;
    constexpr double dq_moving_threshold = 0.05;
    constexpr double ma_alpha = 0.08;

    constexpr double torque_limit = 6.0;

    std::array<double,4> local_cmd;
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        local_cmd = cmd_;
    }
    
    joystick->updateState(targetMode);
    updateFlipperTargetPositions();

    for (int i = 0; i < 4; ++i) {
        if (!flippers_[i]) continue;

        double limited_target;
        if (activeInput_[i] || !stall_locked_[i]) {
            limited_target = local_cmd[i];
        } else {
            limited_target = hold_pos_[i];
        }
        
        double q = flippers_[i]->q();
        double dq = flippers_[i]->dq();

        ma_abs_dq_[i] = (1.0 - ma_alpha) * ma_abs_dq_[i] + ma_alpha * std::abs(dq);

        if (stall_locked_[i]) {
            bool motion_recovered = (ma_abs_dq_[i] > dq_moving_threshold * 2.5);
            bool reached_target = (std::abs(prev_cmd_[i] - q) < error_deadband * 5.0);
            if (motion_recovered || reached_target) {
                stall_locked_[i] = false;
                stall_count_[i] = 0;
                RCLCPP_INFO(node_->get_logger(), "Flipper %d stall lock cleared (motion_recovered=%d,reached_target=%d).", i, (int)motion_recovered, (int)reached_target);
            } else {
                flippers_[i]->u() = 0.02 * (limited_target - q);
                prev_cmd_[i] = limited_target;
                prev_q_[i] = q;
                continue;
            }
        }

        double error = limited_target - q;

        double raw_torque;
        if (std::abs(error) < error_deadband && std::abs(dq) < dq_deadband) {
            raw_torque = 0.0;
        } else if (std::abs(error) < error_deadband) {
            raw_torque = -kd * dq;
        } else {
            raw_torque = kp * error - kd * dq;
        }

        double torque = std::clamp(raw_torque, -torque_limit, torque_limit);

        double torque_abs = std::abs(torque);
        if (ma_abs_dq_[i] < dq_moving_threshold && torque_abs > (torque_stall_ratio * torque_limit)) {
            stall_count_[i]++;
        } else {
            stall_count_[i] = 0;
        }

        if (stall_count_[i] > stall_threshold_count) {
            double now_sec = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (!stall_locked_[i] && (now_sec - last_warn_time_[i] > 1.0)) {
                RCLCPP_WARN(node_->get_logger(),
                    "Flipper %d appears stalled (low motion despite high torque). Engaging safety hold.", i);
                last_warn_time_[i] = now_sec;
            }

            cmd_[i] = q;
            hold_pos_[i] = q;
            prev_cmd_[i] = q;
            prev_q_[i] = q;
            flippers_[i]->u() = 0.02 * (limited_target - q);
            stall_count_[i] = 0;
            stall_locked_[i] = true;
            continue;
        } else {
            flippers_[i]->u() = torque;
        }

        prev_cmd_[i] = limited_target;
        prev_q_[i] = q;
    }

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - lastLogTime_).count() >= 0.5) {
        lastLogTime_ = now;
        for (int i = 0; i < 4; ++i) {
            if (!flippers_[i]) continue;
            RCLCPP_INFO(node_->get_logger(),
                "[Flipper %d] jointId: %d, hold=%.4f, target=%.4f, q=%.4f, torque=%.4f, ma_abs_dq=%.5f, stalled=%s",
                i, flippers_[i]->jointId(), hold_pos_[i], local_cmd[i], flippers_[i]->u(),
                ma_abs_dq_[i],
                stall_locked_[i] ? "YES" : "NO");
        }
    }

    return true;
}

void MobileRobotFlipperController::unconfigure()
{
    if (executor_) {
        executor_->cancel();
        if (executorThread_.joinable()) executorThread_.join();
        executor_->remove_node(node_);
        executor_.reset();
    }
    if (rclcpp::ok()) rclcpp::shutdown();
}

void MobileRobotFlipperController::updateFlipperTargetPositions()
{
    static const double FLIPPER_GAIN = 0.5;

    // Rスティックボタンが押されている場合はフリッパーの位置を揃える
    if(joystick->getButtonState(targetMode, Joystick::R_STICK_BUTTON)){
        double qa = 0.0;
        for(int i=0; i < 4; ++i){
            qa += cmd_[i];
        }
        qa /= static_cast<double>(4);
        double dqmax = 0.05;
        for(int i=0; i < 4; ++i){
            double dq = qa - cmd_[i];
            if(dq > dqmax){
                dq = dqmax;
            } else if(dq < -dqmax){
                dq = -dqmax;
            }
            cmd_[i] += dq;
            cmd_[i] = std::clamp(cmd_[i], flipper_min_limits_[i], flipper_max_limits_[i]);
        }
    } else {
        double pos = joystick->getPosition(targetMode, Joystick::R_STICK_V_AXIS, STICK_THRESH);
        double dq = FLIPPER_GAIN * pos;
        bool FL = joystick->getPosition(targetMode, Joystick::L_TRIGGER_AXIS, STICK_THRESH) > 0.0;
        bool FR = joystick->getPosition(targetMode, Joystick::R_TRIGGER_AXIS, STICK_THRESH) > 0.0;
        bool BL = joystick->getButtonState(targetMode, Joystick::L_BUTTON);
        bool BR = joystick->getButtonState(targetMode, Joystick::R_BUTTON);
        
        if(!FL && !FR && !BL && !BR){
            // 同期モード
            for(int i=0; i < 4; ++i){
                cmd_[i] += dq;
                cmd_[i] = std::clamp(cmd_[i], flipper_min_limits_[i], flipper_max_limits_[i]);
            }
        } else {
            // 個別モード
            // ジョイスティックのボタンとフリッパーの論理的なインデックスを対応させる
            if(FL){
                cmd_[FL_FLIPPER] += dq;
                cmd_[FL_FLIPPER] = std::clamp(cmd_[FL_FLIPPER], flipper_min_limits_[FL_FLIPPER], flipper_max_limits_[FL_FLIPPER]);
            }
            if(FR){
                cmd_[FR_FLIPPER] += dq;
                cmd_[FR_FLIPPER] = std::clamp(cmd_[FR_FLIPPER], flipper_min_limits_[FR_FLIPPER], flipper_max_limits_[FR_FLIPPER]);
            }
            if(BL){
                cmd_[BL_FLIPPER] += dq;
                cmd_[BL_FLIPPER] = std::clamp(cmd_[BL_FLIPPER], flipper_min_limits_[BL_FLIPPER], flipper_max_limits_[BL_FLIPPER]);
            }
            if(BR){
                cmd_[BR_FLIPPER] += dq;
                cmd_[BR_FLIPPER] = std::clamp(cmd_[BR_FLIPPER], flipper_min_limits_[BR_FLIPPER], flipper_max_limits_[BR_FLIPPER]);
            }
        }
    }
}