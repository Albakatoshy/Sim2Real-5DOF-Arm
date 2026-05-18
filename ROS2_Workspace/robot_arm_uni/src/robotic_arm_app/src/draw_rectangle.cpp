#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>

static const std::string PLANNING_GROUP = "arm";
static const double VEL_SCALING         = 0.3;
static const double ACC_SCALING         = 0.2;

// Rectangle size in joint space (radians)
// Increase/decrease to make rectangle bigger/smaller
static const double D_BASE     = 0.3;  // base rotation   → controls WIDTH
static const double D_SHOULDER = 0.2;  // shoulder angle  → controls HEIGHT


// ─────────────────────────────────────────────────────────────
// Plan & execute to joint target
// ─────────────────────────────────────────────────────────────
bool moveToJoints(moveit::planning_interface::MoveGroupInterface& move_group,
                  const std::vector<double>& joints,
                  const rclcpp::Logger& logger,
                  const std::string& label)
{
  move_group.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "  ✓ [%s] executing...", label.c_str());
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "  ✗ [%s] planning FAILED", label.c_str());
  }
  return success;
}


// ─────────────────────────────────────────────────────────────
// Move to safe start configuration
// ─────────────────────────────────────────────────────────────
bool moveToStart(moveit::planning_interface::MoveGroupInterface& move_group,
                 const rclcpp::Logger& logger)
{
  RCLCPP_INFO(logger, "Moving to start pose...");

  // Adjust for your robot
  std::vector<double> start = {
     0.0,   // base
    -0.5,   // shoulder
     0.8,   // upper_arm
     0.0    // gripper_base
  };

  return moveToJoints(move_group, start, logger, "start");
}


// ─────────────────────────────────────────────────────────────
// Draw rectangle using joint space corners
// ─────────────────────────────────────────────────────────────
void drawRectangle(moveit::planning_interface::MoveGroupInterface& move_group,
                   const rclcpp::Logger& logger)
{
  RCLCPP_INFO(logger, "──────────────────────────────────────");
  RCLCPP_INFO(logger, "Drawing rectangle in joint space...");
  RCLCPP_INFO(logger, "  Base delta     : %.2f rad (width)", D_BASE);
  RCLCPP_INFO(logger, "  Shoulder delta : %.2f rad (height)", D_SHOULDER);

  // Read current joint values as the rectangle origin
  std::vector<double> J = move_group.getCurrentJointValues();

  RCLCPP_INFO(logger, "Current joints: [%.2f, %.2f, %.2f, %.2f]",
              J[0], J[1], J[2], J[3]);

  // ── 4 corners + return ───────────────────────────────────
  //
  //   P1 ──── P2     base increases →
  //   │        │
  //   P4 ──── P3     shoulder increases ↓
  //
  // Only joints 0 (base) and 1 (shoulder) move.
  // Joints 2 and 3 stay fixed throughout.

  std::vector<std::pair<std::vector<double>, std::string>> corners = {
    { {J[0],          J[1],            J[2], J[3]}, "P1-start"  },
    { {J[0]+D_BASE,   J[1],            J[2], J[3]}, "P2"        },
    { {J[0]+D_BASE,   J[1]+D_SHOULDER, J[2], J[3]}, "P3"        },
    { {J[0],          J[1]+D_SHOULDER, J[2], J[3]}, "P4"        },
    { {J[0],          J[1],            J[2], J[3]}, "P1-return" },
  };

  // ── Visit each corner ─────────────────────────────────────
  int step = 1;
  for (auto& [joints, label] : corners) {
    RCLCPP_INFO(logger, "Corner %d/5: %s  joints=[%.2f, %.2f, %.2f, %.2f]",
                step++, label.c_str(),
                joints[0], joints[1], joints[2], joints[3]);

    if (!moveToJoints(move_group, joints, logger, label)) {
      RCLCPP_ERROR(logger, "Rectangle aborted at: %s", label.c_str());
      return;
    }
  }

  RCLCPP_INFO(logger, "──────────────────────────────────────");
  RCLCPP_INFO(logger, "✅ Rectangle complete!");
}


// ─────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("draw_rectangle", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto logger = node->get_logger();

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(VEL_SCALING);
  move_group.setMaxAccelerationScalingFactor(ACC_SCALING);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);

  RCLCPP_INFO(logger, "Planning frame : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector   : %s", move_group.getEndEffectorLink().c_str());

  if (!moveToStart(move_group, logger)) {
    executor.cancel(); spinner.join();
    rclcpp::shutdown(); return 1;
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  drawRectangle(move_group, logger);

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}