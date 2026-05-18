#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <regex>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>

class VoiceToMoveIt : public rclcpp::Node {
public:
    VoiceToMoveIt(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("voice_to_moveit", options), ready_(false)
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/voice_commands", 10,
            std::bind(&VoiceToMoveIt::voice_callback, this, std::placeholders::_1),
            sub_opts);

        // ✅ Use a one-shot timer instead of a raw thread in the constructor.
        //    By the time the timer fires, shared_from_this() is safe to call.
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                init_timer_->cancel();  // fire only once
                init_thread_ = std::thread(&VoiceToMoveIt::initialize_moveit, this);
            });

        RCLCPP_INFO(this->get_logger(), "C++ Brain is initializing...");
    }

    ~VoiceToMoveIt() {
        if (init_thread_.joinable()) init_thread_.join();
    }

private:
    void initialize_moveit() {
        // shared_from_this() is now safe — node is fully constructed
        
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        move_group_arm_->setPlanningTime(5.0);
        move_group_gripper_->setPlanningTime(5.0);

        ready_.store(true);
        RCLCPP_INFO(this->get_logger(), " C++ Brain Ready! Listening for commands...");
    }

    void voice_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!ready_.load()) {
            RCLCPP_WARN(this->get_logger(), " MoveIt not ready yet, ignoring command.");
            return;
        }

        std::string command = msg->data;
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);
        RCLCPP_INFO(this->get_logger(), " Received: '%s'", command.c_str());

        if (command.find("home") != std::string::npos) {
            execute_named_pose("home");
            return;
        }

        if (command.find("open") != std::string::npos) {
            move_gripper(0.0);
            return;
        }

        if (command.find("close") != std::string::npos) {
            move_gripper(0.8);
            return;
        }

        if (command.find("move")   != std::string::npos ||
            command.find("rotate") != std::string::npos ||
            command.find("tilt")   != std::string::npos ||
            command.find("nudge")  != std::string::npos)
        {
            handle_joint_command(command);
        }
    }

    void handle_joint_command(const std::string& command) {
        int joint_idx = -1;
        if      (command.find("base")     != std::string::npos) joint_idx = 0;
        else if (command.find("shoulder") != std::string::npos) joint_idx = 1;
        else if (command.find("upper")    != std::string::npos) joint_idx = 2;
        else if (command.find("wrist")    != std::string::npos) joint_idx = 3;

        if (joint_idx == -1) {
            RCLCPP_WARN(this->get_logger(), " No joint name found in command.");
            return;
        }

        // Correct for joints whose positive URDF direction is visually "backward"
        static const std::vector<int> flipped_joints = {1};  // shoulder is flipped
        bool is_flipped = std::find(flipped_joints.begin(), flipped_joints.end(), joint_idx)
                          != flipped_joints.end();

        double degrees = extract_number(command);
        if (degrees == 0.0) degrees = 15.0;

        double delta = degrees * (M_PI / 180.0);

        static const std::vector<std::string> neg_words =
            {"back", "backward", "down", "right", "negative"};
        bool is_negative = false;
        for (const auto& w : neg_words) {
            if (command.find(w) != std::string::npos) {
                is_negative = true;
                break;
            }
        }

        if (is_negative) delta = -delta;
        if (is_flipped)  delta = -delta;  // compensate for URDF axis flip

        nudge_joint(joint_idx, delta);
    }

    void nudge_joint(int index, double delta) {
        std::vector<double> positions;
        const int max_attempts = 10;
        for (int i = 0; i < max_attempts; ++i) {
            move_group_arm_->setStartStateToCurrentState();
            positions = move_group_arm_->getCurrentJointValues();
            if (!positions.empty()) break;

            RCLCPP_INFO(this->get_logger(),
                "⌛ Waiting for joint state... (%d/%d)", i + 1, max_attempts);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        if (positions.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "Could not get joint state. Is the simulation running?");
            return;
        }

        if (index >= static_cast<int>(positions.size())) {
            RCLCPP_ERROR(this->get_logger(),
                "Joint index %d out of range (arm has %zu joints).",
                index, positions.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Joint %d: %.4f → %.4f rad",
            index, positions[index], positions[index] + delta);

        positions[index] += delta;
        move_group_arm_->setJointValueTarget(positions);
        plan_and_execute(move_group_arm_);
    }

    void move_gripper(double value) {
        auto positions = move_group_gripper_->getCurrentJointValues();
        if (positions.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Gripper state unavailable.");
            return;
        }
        std::fill(positions.begin(), positions.end(), value);
        move_group_gripper_->setJointValueTarget(positions);
        plan_and_execute(move_group_gripper_);
    }

    void execute_named_pose(const std::string& name) {
        move_group_arm_->setNamedTarget(name);
        plan_and_execute(move_group_arm_);
    }

    void plan_and_execute(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan found — executing...");
            group->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

    double extract_number(const std::string& text) {
        std::smatch match;
        std::regex re(R"((\d+(\.\d+)?))");
        if (std::regex_search(text, match, re))
            return std::stod(match.str(1));
        return 0.0;
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    std::thread init_thread_;
    std::atomic<bool> ready_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoiceToMoveIt>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}