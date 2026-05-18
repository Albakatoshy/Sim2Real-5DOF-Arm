#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moveit.planning import MoveItPy
import math

class VoiceToMoveIt(Node):
    def __init__(self):
        super().__init__('voice_to_moveit')
        
        self.get_logger().info("🤖 Abdalrahman, the MoveIt Brain is initializing...")
        
        # Load MoveItPy - This connects to your URDF/SRDF automatically
        self.moveit = MoveItPy(node_name="voice_to_moveit")
        self.arm = self.moveit.get_planning_component("arm")
        self.gripper = self.moveit.get_planning_component("gripper")
        
        self.subscription = self.create_subscription(
            String, '/voice_commands', self.voice_callback, 10)
            
        self.get_logger().info("🚀 Ready! Listening for your directional commands...")

    def voice_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"📥 Received: '{command}'")

        # 1. Named Targets (from your SRDF)
        if "home" in command:
            self.execute_named_pose("home")
            return

        # 2. Basic Gripper Actions
        if "open" in command and "gripper" in command:
            self.move_gripper_fingers(0.0) # Open position
            return
        elif "close" in command and "gripper" in command:
            self.move_gripper_fingers(0.8) # Adjust based on your CAD geometry
            return

        # 3. Directional Nudges (Forward, Backward, Up, Down, Left, Right)
        if any(word in command for word in ["move", "rotate", "tilt", "nudge"]):
            
            # Extract degrees if spoken, otherwise use a default 15° nudge
            degrees = self.extract_number(command)
            if degrees is None:
                degrees = 15.0 
            
            # MoveIt uses Radians: $rad = deg \times (\pi / 180)$
            delta_radians = degrees * (math.pi / 180.0)
            
            # Map directional words to negative values
            neg_keywords = ["back", "backward", "down", "right", "negative", "minus"]
            if any(word in command for word in neg_keywords):
                delta_radians = -delta_radians

            # Map keyword to your specific Joints
            # Indices: 0:base, 1:shoulder, 2:upper, 3:wrist, 4:gripper_base
            joint_idx = None
            if "base" in command: joint_idx = 0
            elif "shoulder" in command: joint_idx = 1
            elif "upper" in command: joint_idx = 2
            elif "wrist" in command: joint_idx = 3
            elif "gripper" in command: joint_idx = 4 # Nudges the gripper base link
            
            if joint_idx is not None:
                self.nudge_joint(joint_idx, delta_radians)
            else:
                self.get_logger().warn("Command heard, but I don't know which joint to move!")

    # --- Utility Functions ---

    def extract_number(self, text):
        """Strips symbols like ° and pulls out the first number."""
        clean_text = text.replace('°', '').replace('degrees', '').replace('degree', '')
        for word in clean_text.split():
            try:
                return float(word)
            except ValueError:
                continue
        return None

    def nudge_joint(self, index, delta):
        """Relative movement logic."""
        state = self.moveit.get_robot_model().create_robot_state()
        state.update()
        positions = state.get_joint_group_positions("arm")
        
        # Apply the nudge to current position
        positions[index] += delta
        
        state.set_joint_group_positions("arm", positions)
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=state)
        self.plan_and_run(self.arm)

    def move_gripper_fingers(self, value):
        state = self.moveit.get_robot_model().create_robot_state()
        state.update()
        state.set_joint_group_positions("gripper", [value])
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(robot_state=state)
        self.plan_and_run(self.gripper)

    def execute_named_pose(self, pose):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=pose)
        self.plan_and_run(self.arm)

    def plan_and_run(self, component):
        plan = component.plan()
        if plan:
            self.get_logger().info("✅ Plan found. Executing movement...")
            self.moveit.execute(plan.trajectory, controllers=[])
        else:
            self.get_logger().error("❌ Planning failed! Check for collisions.")

def main():
    rclpy.init()
    node = VoiceToMoveIt()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()