#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import TransitionEvent

class WaitTopicCreation(Node):
    def __init__(self):
        super().__init__('wait_topic_creation')
        self.declare_parameter('robot_type', 'drone')
        self.robot_type = self.get_parameter('robot_type').value
        self.declare_parameter('summit_id', 1)
        self.summit_id = str(self.get_parameter('summit_id').value)
        self.declare_parameter('ranger_id', 1)
        self.ranger_id = str(self.get_parameter('ranger_id').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        if self.robot_type == "drone":
            self.get_logger().info("Waiting for topic /px4_1/fmu/out/vehicle_odometry to be created...")
            self.sub = self.create_subscription(
                VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', self.drone_callback, qos_profile)
        elif self.robot_type == "summit":
            self.get_logger().info(f"Waiting for topic /summit_xl_{self.summit_id}/robotnik_base_controller/odom to be created...")
            self.sub = self.create_subscription(
                Odometry, f'/summit_xl_{self.summit_id}/robotnik_base_controller/odom', self.summit_callback, qos_profile)
        elif self.robot_type == "ranger":
            self.get_logger().info(f"Waiting for topic /ranger_mini_{self.ranger_id}/forward_position_controller/transition_event to be created...")
            self.sub = self.create_subscription(
                TransitionEvent, f'/ranger_mini_{self.ranger_id}/forward_position_controller/transition_event', self.ranger_callback, 10)

    def drone_callback(self, msg):
        self.get_logger().info(f"Topic /px4_1/fmu/out/vehicle_odometry created")
        raise SystemExit
    
    def summit_callback(self, msg):
        self.get_logger().info(f"Topic /summit_xl_{self.summit_id}/robotnik_base_controller/odom created")
        raise SystemExit
    
    def ranger_callback(self, msg):
        self.get_logger().info(f"Topic /ranger_mini_{self.ranger_id}/forward_position_controller/transition_event created")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = WaitTopicCreation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
