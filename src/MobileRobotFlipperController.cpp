// MobileRobotFlipperController.cpp (patched: per-joint torque limits, tuned stall logic)
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
    std::array<double,4> cmd_{0.0,0.0,0.0,0.0};
    std::array<double,4> hold_pos_{0.0,0.0,0.0,0.0};
    std::mutex cmdMutex_;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor_;
    std::thread executorThread_;

    std::array<bool,4> activeInput_{false,false,false,false};
    std::array<double,4> prev_cmd_{0.0,0.0,0.0,0.0};
    std::array<double,4> prev_q_{0.0,0.0,0.0,0.0};
    std::array<double,4> ma_abs_dq_{0.0,0.0,0.0,0.0};

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

    std::array<double,4> torque_limit_ = {8.0, 8.0, 12.0, 12.0};
    std::array<double,4> torque_stall_ratio_ = {0.8, 0.8, 0.95, 0.95};
    std::array<int,4> stall_threshold_count_ = {20, 20, 60, 60};
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

            constexpr double joint_limit         = 1.57;
            constexpr double deadband_enter      = 0.02;
            constexpr double deadband_exit       = 0.01;

            for (size_t i = 0; i < msg->name.size(); ++i) {
                auto it = jointNameMap.find(msg->name[i]);
                if (it == jointNameMap.end()) continue;
                int idx = it->second;
                if (i >= msg->position.size()) continue;
                double offset = msg->position[i];

                if (std::abs(offset) > deadband_enter) {
                    activeInput_[idx] = true;
                    double target = hold_pos_[idx] + std::clamp(offset, -joint_limit, joint_limit);
                    target = std::clamp(target, -joint_limit, joint_limit);
                    cmd_[idx] = target;
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
    auto body = io->body();
    for (int i = 0; i < 4; ++i) {
        flippers_[i] = nullptr;
    }

    flippers_[0] = body->joint("FlipperLeftJoint");
    flippers_[1] = body->joint("FlipperRightJoint");
    flippers_[2] = body->joint("FlipperRearLeftJoint");
    flippers_[3] = body->joint("FlipperRearRightJoint");

    for (int i = 0; i < 4; ++i) {
        if (!flippers_[i]) {
            continue;
        }
        flippers_[i]->setActuationMode(Link::JointTorque);
        io->enableInput(flippers_[i], Link::JointVelocity);
        io->enableOutput(flippers_[i], Link::JointTorque);

        hold_pos_[i] = flippers_[i]->q();
        cmd_[i] = hold_pos_[i];
        prev_cmd_[i] = cmd_[i];
        prev_q_[i] = cmd_[i];
        ma_abs_dq_[i] = 0.0;
        stall_count_[i] = 0;
        stall_locked_[i] = false;
    }

    return true;
}

bool MobileRobotFlipperController::control()
{
    constexpr double kp = 5.0;
    constexpr double kd = 3.0;
    constexpr double error_deadband = 0.005;
    constexpr double dq_deadband = 0.02;

    const double max_step_per_cycle = 0.10;

    constexpr double dq_moving_threshold = 0.005;
    constexpr double ma_alpha = 0.08;

    std::array<double,4> local_cmd;
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        local_cmd = cmd_;
    }

    for (int i = 0; i < 4; ++i) {
        if (!flippers_[i]) continue;

        double target = local_cmd[i];
        double limited_target = std::clamp(target, prev_cmd_[i] - max_step_per_cycle, prev_cmd_[i] + max_step_per_cycle);

        double q = flippers_[i]->q();
        double dq = flippers_[i]->dq();

        ma_abs_dq_[i] = (1.0 - ma_alpha) * ma_abs_dq_[i] + ma_alpha * std::abs(dq);

        if (stall_locked_[i]) {
            bool motion_recovered = (ma_abs_dq_[i] > dq_moving_threshold * 2.5);
            bool reached_target = (std::abs(prev_cmd_[i] - q) < error_deadband * 5.0);
            if (motion_recovered || reached_target) {
                stall_locked_[i] = false;
                stall_count_[i] = 0;
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

        double torque_limit = torque_limit_[i];
        double torque = std::clamp(raw_torque, -torque_limit, torque_limit);

        double torque_abs = std::abs(torque);
        double torque_stall_ratio = torque_stall_ratio_[i];
        int stall_threshold_count = stall_threshold_count_[i];

        if (ma_abs_dq_[i] < dq_moving_threshold && torque_abs > (torque_stall_ratio * torque_limit)) {
            stall_count_[i]++;
        } else {
            stall_count_[i] = 0;
        }

        if (stall_count_[i] > stall_threshold_count) {
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

