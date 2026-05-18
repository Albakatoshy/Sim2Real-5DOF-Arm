import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_box_pose', 10)
        self.br = CvBridge()

        # ── Calibration ──────────────────────────────────────────────
        # Tune these so that pixel coordinates map to real robot metres.
        self.image_width   = 640      # px  (match your camera resolution)
        self.x_fixed       = 0.45     # m   distance forward (depth)
        self.y_scale       = 0.001    # m/px lateral scale
        self.z_fixed       = 0.025    # m   half box height (table level)
        # ─────────────────────────────────────────────────────────────

        self.get_logger().info("Box Detector started — waiting for /camera/image …")

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red spans both ends of the HSV hue circle
        mask  = cv2.inRange(hsv, np.array([  0, 100, 100]),
                                  np.array([ 10, 255, 255]))
        mask |= cv2.inRange(hsv, np.array([160, 100, 100]),
                                  np.array([180, 255, 255]))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        # ── Pick the LARGEST red blob ─────────────────────────────────
        best = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500 and area > best_area:   # ignore tiny noise
                best      = cnt
                best_area = area

        if best is not None:
            x, y, w, h = cv2.boundingRect(best)
            cx = x + w // 2
            cy = y + h // 2

            # Draw bounding box and crosshair
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"({cx}, {cy})", (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            self.get_logger().info(
                f"TARGET FOUND — pixel ({cx}, {cy}), area={best_area:.0f}")

            # ── Publish 3-D pose estimate ─────────────────────────────
            pose_msg = PoseStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"

            pose_msg.pose.position.x = self.x_fixed
            pose_msg.pose.position.y = (cx - self.image_width // 2) * self.y_scale
            pose_msg.pose.position.z = self.z_fixed

            # Keep gripper horizontal (no rotation)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            self.pose_pub.publish(pose_msg)
        else:
            self.get_logger().debug("No red box visible.")

        cv2.imshow("Robot Vision", frame)
        cv2.waitKey(1)


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()