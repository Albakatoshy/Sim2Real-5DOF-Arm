// box_picker.cpp  —  fixed version
//
// Changes vs previous version:
//   1. wait_for_joint_states() — spins until /joint_states has a real timestamp
//   2. Cooldown timer — after any failure, ignores poses for COOLDOWN_SEC seconds
//   3. set_gripper() retries up to 5x with 300 ms gaps

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// ── Tunable constants ─────────────────────────────────────────────────────
static constexpr double APPROACH_HEIGHT  = 0.12;
static constexpr double LIFT_HEIGHT      = 0.20;
static constexpr double GRIPPER_OPEN     = 0.0;
static constexpr double GRIPPER_CLOSE    = 0.8;
static constexpr int    JOINT_STATE_WAIT = 30;    // seconds
static constexpr double COOLDOWN_SEC     = 8.0;
// ─────────────────────────────────────────────────────────────────────────

class BoxPicker : public rclcpp::Node
{
public:
    explicit BoxPicker(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : Node("box_picker", opts), ready_(false), picking_(false), js_ready_(false),
      last_attempt_(this->now() - rclcpp::Duration::from_seconds(COOLDOWN_SEC + 1))
    {
        cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/detected_box_pose", 10,
            std::bind(&BoxPicker::pose_callback, this, std::placeholders::_1),
            sub_opts);

        // Monitor joint_states to know when the robot is actually running
        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                if (msg->header.stamp.sec > 0)
                    js_ready_.store(true);
            });

        init_timer_ = this->create_wall_timer(500ms, [this]() {
            init_timer_->cancel();
            init_thread_ = std::thread(&BoxPicker::init_moveit, this);
        });

        RCLCPP_INFO(get_logger(), "BoxPicker: initialising ...");
    }

    ~BoxPicker() { if (init_thread_.joinable()) init_thread_.join(); }

private:
    // ── MoveIt init ───────────────────────────────────────────────────────
    void init_moveit()
    {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                   shared_from_this(), "arm");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                       shared_from_this(), "gripper");

        arm_->setPlanningTime(10.0);
        gripper_->setPlanningTime(5.0);

        ready_.store(true);
        RCLCPP_INFO(get_logger(), "BoxPicker ready — listening on /detected_box_pose");
    }

    // ── Pose subscriber callback ──────────────────────────────────────────
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!ready_.load()) return;

        double elapsed = (this->now() - last_attempt_).seconds();
        if (elapsed < COOLDOWN_SEC) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Cooldown: %.1f / %.1f s", elapsed, COOLDOWN_SEC);
            return;
        }

        if (picking_.exchange(true)) return;   // already running

        last_attempt_ = this->now();

        std::thread([this, msg]() {
            execute_pick(msg->pose);
            picking_.store(false);
        }).detach();
    }

    // ── Block until joint_states has a real timestamp ─────────────────────
    bool wait_for_joint_states()
    {
        RCLCPP_INFO(get_logger(), "Waiting for /joint_states with valid timestamp...");
        for (int i = 0; i < JOINT_STATE_WAIT * 10; ++i) {
            if (js_ready_.load()) {
                RCLCPP_INFO(get_logger(), "Joint states OK — proceeding.");
                return true;
            }
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_ERROR(get_logger(),
            "Timed out waiting for /joint_states.\n"
            "  -> Is Gazebo / the robot driver running?\n"
            "  -> Check: ros2 topic echo /joint_states");
        return false;
    }

    // ── Full pick sequence ────────────────────────────────────────────────
    void execute_pick(const geometry_msgs::msg::Pose & box_pose)
    {
        if (!wait_for_joint_states()) return;

        // 1. Open
        RCLCPP_INFO(get_logger(), "[1/5] Opening gripper...");
        if (!set_gripper(GRIPPER_OPEN)) return;

        // 2. Approach
        RCLCPP_INFO(get_logger(), "[2/5] Approaching (%.2f m above box)...",
            APPROACH_HEIGHT);
        auto approach = box_pose;
        approach.position.z += APPROACH_HEIGHT;
        approach.orientation  = down_quat();
        if (!move_to_pose(approach)) {
            RCLCPP_ERROR(get_logger(), "Approach failed — aborting pick.");
            return;
        }

        // 3. Descend
        RCLCPP_INFO(get_logger(), "[3/5] Descending to box...");
        auto grasp = box_pose;
        grasp.orientation = down_quat();
        if (!move_to_pose(grasp)) {
            RCLCPP_ERROR(get_logger(), "Descent failed — aborting pick.");
            return;
        }

        // 4. Close
        RCLCPP_INFO(get_logger(), "[4/5] Closing gripper...");
        if (!set_gripper(GRIPPER_CLOSE)) return;
        std::this_thread::sleep_for(500ms);

        // 5. Lift
        RCLCPP_INFO(get_logger(), "[5/5] Lifting...");
        auto lift = grasp;
        lift.position.z += LIFT_HEIGHT;
        if (!move_to_pose(lift)) {
            RCLCPP_ERROR(get_logger(), "Lift failed.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Pick complete!");
    }

    // ── Helpers ───────────────────────────────────────────────────────────
    bool move_to_pose(const geometry_msgs::msg::Pose & target)
    {
        arm_->setStartStateToCurrentState();
        arm_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (ok) arm_->execute(plan);
        arm_->clearPoseTargets();
        return ok;
    }

    bool set_gripper(double value)
    {
        for (int attempt = 1; attempt <= 5; ++attempt) {
            auto pos = gripper_->getCurrentJointValues();
            if (!pos.empty()) {
                std::fill(pos.begin(), pos.end(), value);
                gripper_->setJointValueTarget(pos);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    gripper_->execute(plan);
                    return true;
                }
            }
            RCLCPP_WARN(get_logger(), "Gripper attempt %d/5 failed, retrying...", attempt);
            std::this_thread::sleep_for(300ms);
        }
        RCLCPP_ERROR(get_logger(), "Gripper failed after 5 attempts.");
        return false;
    }

    // Gripper pointing straight down (90 deg pitch around Y axis)
    geometry_msgs::msg::Quaternion down_quat()
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0; q.y = 0.7071068; q.z = 0.0; q.w = 0.7071068;
        return q;
    }

    // ── Members ───────────────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    js_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::thread                  init_thread_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    std::atomic<bool> ready_;
    std::atomic<bool> picking_;
    std::atomic<bool> js_ready_;
    rclcpp::Time      last_attempt_;
};

// ── main ──────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoxPicker>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}