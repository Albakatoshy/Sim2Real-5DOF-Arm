import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_py.modeling_components import MoveGroupInterface

class RobotMover(Node):
    def __init__(self):
        super().__init__('mover_node')
        
        # 1. Initialize MoveIt Python Interface
        self.moveit_config = MoveItConfigsBuilder("robotic_arm").to_moveit_configs()
        self.arm = MoveGroupInterface("arm", self.moveit_config)
        self.gripper = MoveGroupInterface("gripper", self.moveit_config)

        # 2. Subscriber for target position (from vision node)
        self.target_sub = self.create_subscription(
            PoseStamped, '/detected_box_pose', self.execute_pick, 10)
        
        self.get_logger().info("Mover Node Initialized. Waiting for detection...")

    def execute_pick(self, msg):
        target_pose = msg.pose
        
        # --- PHASE 1: PRE-GRASP (Hover above box) ---
        hover_pose = target_pose
        hover_pose.position.z += 0.1  # Stay 10cm above
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(hover_pose)
        self.get_logger().info("Planning to hover pose...")
        self.arm.plan().execute()

        # --- PHASE 2: OPEN GRIPPER ---
        self.gripper.set_named_target("open")
        self.gripper.plan().execute()

        # --- PHASE 3: APPROACH ---
        self.arm.set_pose_target(target_pose)
        self.arm.plan().execute()

        # --- PHASE 4: GRIP ---
        self.gripper.set_named_target("closed")
        self.gripper.plan().execute()

        # --- PHASE 5: LIFT ---
        hover_pose.position.z += 0.1
        self.arm.set_pose_target(hover_pose)
        self.arm.plan().execute()

def main():
    rclpy.init()
    node = RobotMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()