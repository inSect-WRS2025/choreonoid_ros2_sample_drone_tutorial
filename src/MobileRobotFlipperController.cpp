// MobileRobotFlipperController.cpp (updated: stall unlock logic added)
#include <cnoid/SimpleController>
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

using namespace cnoid;

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

    std::chrono::steady_clock::time_point lastLogTime_;
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

    // Subscriber: msg.position は「瞬時オフセット」（joy 側が中立なら 0 を送る前提）
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "flipper_command", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg){
            std::lock_guard<std::mutex> lock(cmdMutex_);

            constexpr double joint_limit         = 1.57;
            constexpr double deadband_enter      = 0.02; // オフセットがこれより大きければ入力あり
            constexpr double deadband_exit       = 0.01; // これ以下なら入力終了とみなす

            for (size_t i = 0; i < msg->name.size(); ++i) {
                auto it = jointNameMap.find(msg->name[i]);
                if (it == jointNameMap.end()) continue;
                int idx = it->second;
                if (i >= msg->position.size()) continue;
                double offset = msg->position[i]; // 瞬時オフセット（joy 側）

                if (std::abs(offset) > deadband_enter) {
                    // 入力開始または継続
                    activeInput_[idx] = true;
                    double target = hold_pos_[idx] + std::clamp(offset, -joint_limit, joint_limit);
                    target = std::clamp(target, -joint_limit, joint_limit);
                    cmd_[idx] = target;
                } else {
                    // オフセットが小さい -> 未入力と判断
                    if (activeInput_[idx]) {
                        // 入力直後の終了判定: 現在の cmd を hold に採用
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
    // 10度をラジアンに変換する定数
    // 10.0 * M_PI / 180.0 = 0.174532925...
    constexpr double DEG10_TO_RAD = 10.0 * M_PI / 180.0; 

    auto body = io->body();
    for (int i = 0; i < 4; ++i) {
        flippers_[i] = nullptr;
    }

    // JointNameMap: {"FlipperLeftJoint", 0}, {"FlipperRightJoint", 1}, {"FlipperRearLeftJoint", 2}, {"FlipperRearRightJoint", 3}
    flippers_[0] = body->joint("FlipperLeftJoint");
    flippers_[1] = body->joint("FlipperRightJoint");
    flippers_[2] = body->joint("FlipperRearLeftJoint");
    flippers_[3] = body->joint("FlipperRearRightJoint");

    // 初期目標角度の定義
    // Flipper 0: -10 deg, Flipper 1: +10 deg, Flipper 2: -10 deg, Flipper 3: +10 deg
    const std::array<double, 4> initial_targets = {
        -DEG10_TO_RAD,  // FlipperLeftJoint (0)
        DEG10_TO_RAD,   // FlipperRightJoint (1)
        -DEG10_TO_RAD,  // FlipperRearLeftJoint (2)
        DEG10_TO_RAD    // FlipperRearRightJoint (3)
    };

    for (int i = 0; i < 4; ++i) {
        if (!flippers_[i]) {
            RCLCPP_WARN(node_->get_logger(), "Joint %d not found.", i);
            continue;
        }
        // トルク制御モード
        flippers_[i]->setActuationMode(Link::JointTorque);
        // 環境互換のため JointVelocity を enable（q,dq は取得できます）
        io->enableInput(flippers_[i], Link::JointVelocity);
        io->enableOutput(flippers_[i], Link::JointTorque);

        // ** ここから変更 **
        // 初期保持位置と目標位置を指定角度でセット
        hold_pos_[i] = initial_targets[i]; // 指定された初期目標位置
        cmd_[i] = initial_targets[i];      // 指定された初期目標位置
        // ** 変更終わり **

        prev_cmd_[i] = cmd_[i];
        prev_q_[i] = flippers_[i]->q(); // 現在角度は実際のフリッパーから取得
        ma_abs_dq_[i] = 0.0;
        stall_count_[i] = 0;
        stall_locked_[i] = false;

        RCLCPP_INFO(node_->get_logger(), "[Init][%d] hold_q=%.4f, init_target=%.4f", i, hold_pos_[i], cmd_[i]);
    }

    return true;
}

bool MobileRobotFlipperController::control()
{
    // PD 控制パラメータ（保守的）
    constexpr double kp = 7.0;
    constexpr double kd = 4.0;
    constexpr double error_deadband = 0.005;
    constexpr double dq_deadband = 0.02;

    // rate limit step per cycle（ラジアン）
    const double max_step_per_cycle = 0.05;

    // stall 判定パラメータ
    constexpr double torque_stall_ratio = 0.8;    // torque が limit の何割なら懸念
    constexpr int stall_threshold_count = 20;     // 連続サイクル数（少し短め）
    constexpr double dq_moving_threshold = 0.05; // dq の移動平均がこの以下なら停止とみなす
    constexpr double ma_alpha = 0.08;             // dq 移動平均の係数

    constexpr double torque_limit = 6.0; // 要調整

    std::array<double,4> local_cmd;
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        local_cmd = cmd_;
    }

    for (int i = 0; i < 4; ++i) {
        if (!flippers_[i]) continue;

        double target = local_cmd[i];

        // 目標のレート制限（滑らか化）
        double limited_target = std::clamp(target, prev_cmd_[i] - max_step_per_cycle, prev_cmd_[i] + max_step_per_cycle);

        double q = flippers_[i]->q();
        double dq = flippers_[i]->dq();

        // 移動平均で dq の絶対値を追う（ノイズ吸収）
        ma_abs_dq_[i] = (1.0 - ma_alpha) * ma_abs_dq_[i] + ma_alpha * std::abs(dq);

        // ** stall_locked の解除判定 **
        // 動きが回復したらロック解除する（ma_abs_dq が大きい or 目標と現在角が一致）
        if (stall_locked_[i]) {
            bool motion_recovered = (ma_abs_dq_[i] > dq_moving_threshold * 2.5);
            bool reached_target = (std::abs(prev_cmd_[i] - q) < error_deadband * 5.0);
            if (motion_recovered || reached_target) {
                stall_locked_[i] = false;
                stall_count_[i] = 0;
                RCLCPP_INFO(node_->get_logger(), "Flipper %d stall lock cleared (motion_recovered=%d,reached_target=%d).", i, (int)motion_recovered, (int)reached_target);
                // fallthrough to normal operation
            } else {
                // まだロック中：極小トルクで保持して進まないようにする
                flippers_[i]->u() = 0.02 * (limited_target - q); // ごく弱く位置保持
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

        // スタック判定（高トルクかつほとんど動いていない）
        double torque_abs = std::abs(torque);
        if (ma_abs_dq_[i] < dq_moving_threshold && torque_abs > (torque_stall_ratio * torque_limit)) {
            stall_count_[i]++;
        } else {
            stall_count_[i] = 0;
        }

        if (stall_count_[i] > stall_threshold_count) {
            // クールダウンで過剰ログを防ぐ
            double now_sec = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (!stall_locked_[i] && (now_sec - last_warn_time_[i] > 1.0)) {
                RCLCPP_WARN(node_->get_logger(),
                    "Flipper %d appears stalled (low motion despite high torque). Engaging safety hold.", i);
                last_warn_time_[i] = now_sec;
            }

            // 一度安全措置として目標を現在角にリセットし、トルクを抑え、ロックする
            cmd_[i] = q;
            hold_pos_[i] = q;
            prev_cmd_[i] = q;
            prev_q_[i] = q;
            flippers_[i]->u() = 0.02 * (limited_target - q); // ごく弱く保持
            stall_count_[i] = 0;
            stall_locked_[i] = true; // ロックして同じ警告をスパムしない
            continue;
        } else {
            // 通常運転フロー
            flippers_[i]->u() = torque;
        }

        // prev 更新
        prev_cmd_[i] = limited_target;
        prev_q_[i] = q;
    }

    // ログ出力（0.5秒ごと）
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - lastLogTime_).count() >= 0.5) {
        lastLogTime_ = now;
        for (int i = 0; i < 4; ++i) {
            if (!flippers_[i]) continue;
            RCLCPP_INFO(node_->get_logger(),
                "[Flipper %d] hold=%.4f, target=%.4f, q=%.4f, torque=%.4f, ma_abs_dq=%.5f, stalled=%s",
                i, hold_pos_[i], local_cmd[i], flippers_[i]->q(), flippers_[i]->u(),
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
