import rclpy
from rclpy.node import Node
from robotic_arm.object_detector import VisionObjectDetector
from robotic_arm.inverse_kinematics import TrajectoryPublisher
from custom_message.msg import DetectedObjectsStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration


class ArmCommander(Node):
    def __init__(self):
        super().__init__("robotic_arm_commander")
        self.arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "gripper_hand_joint",
        ]

        # Initialize publishers for controlling the robotic arm
        self.joint_command_publisher = self.create_publisher(
            JointState, "joint_commands", 10
        )
        self.arm_pose_publisher = self.create_publisher(Pose, "arm_pose", 10)

        # Internal variables to store current joint state and detected objects
        self.current_joint_state = JointState()
        self.detected_objects = []

        self.vision_detector = VisionObjectDetector()
        self.trajectory_publisher = TrajectoryPublisher()

        self.subscription = self.create_subscription(
            DetectedObjectsStamped,
            "/object_detection",
            self.object_detection_callback,
            qos_profile=10,
        )
        self.subscription

    def object_detection_callback(self, msg):
        if msg.detected_objects:
            self.detected_objects.append(msg)

    def move_to_neutral(self):
        # Example function to move the arm to a neutral position
        joint_commands = JointState()
        joint_commands.name = self.arm_joint_names
        joint_commands.position = [0.0] * len(self.arm_joint_names)
        self.joint_command_publisher.publish(joint_commands)
        self.get_logger().info("Moving to neutral position.")

    def select_random_object(self):
        if self.detected_objects:
            selected_object = self.detected_objects.pop(0)
            self.get_logger().info(f"Selected object: {selected_object}")
            return selected_object
        else:
            return None

    def move_object(self, x, y, z, color):
        self.trajectory_publisher.inverse_kinematics_solution(x, y, z, "o")

        # Send a trajectory to move the object
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.trajectory_publisher.joints
        point = JointTrajectoryPoint()
        point.positions = self.trajectory_publisher.goal_positions
        point.time_from_start = Duration(sec=1)
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publisher.trajectory_publihser.publish(bazu_trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ArmCommander()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
