#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

import numpy as np

class TfPublisher(Node):
    """Create tfs between Rviz's fixed frame and robots to synchronize their movement between Rviz & Gazebo (allowing to see their movements in Rviz)"""
    def __init__(self):
        super().__init__('tf_publisher')
        
        # Configure QoS profile for publishing and subscribing to PX4's topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameters
        self.declare_parameter('nb_drones', 0)

        self.broadcaster = StaticTransformBroadcaster(self)

        # Create static tf between world (fixed frame in Rviz) and summit's odom frame
        self.static_tf_summit = TransformStamped()
        self.static_tf_summit.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_summit.header.frame_id = 'world'
        self.static_tf_summit.child_frame_id = 'summit_xl_1/summit_xl_odom'

        # Translations (adapt to summit's initial pose)
        self.static_tf_summit.transform.translation.x = 0.0
        self.static_tf_summit.transform.translation.y = 1.0
        self.static_tf_summit.transform.translation.z = 0.0

        # Rotations (not any here)
        self.static_tf_summit.transform.rotation.x = 0.0
        self.static_tf_summit.transform.rotation.y = 0.0
        self.static_tf_summit.transform.rotation.z = 0.0
        self.static_tf_summit.transform.rotation.w = 1.0

        # Create static tf between world (fixed frame in Rviz) and ranger's base frame 
        # (should rather use odom frame to synchonize movements between Rviz & Gazebo, but there is no such frame for this robot for now)
        self.static_tf_ranger = TransformStamped()
        self.static_tf_ranger.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_ranger.header.frame_id = 'world'
        self.static_tf_ranger.child_frame_id = 'ranger_mini_1/base_link'

        # Translations (adapt to summit's initial pose)
        self.static_tf_ranger.transform.translation.x = 0.0
        self.static_tf_ranger.transform.translation.y = 2.0
        self.static_tf_ranger.transform.translation.z = 0.0

        # Rotations (not any here)
        self.static_tf_ranger.transform.rotation.x = 0.0
        self.static_tf_ranger.transform.rotation.y = 0.0
        self.static_tf_ranger.transform.rotation.z = 0.0
        self.static_tf_ranger.transform.rotation.w = 1.0
        
        # If there is one drone simulated, subscribe to its odometry topic, otherwise simply publish the static tfs
        if self.get_parameter('nb_drones').value > 0:
            self.subscription = self.create_subscription(
                VehicleOdometry,
                '/px4_1/fmu/out/vehicle_odometry',
                self.handle_odom,
                qos_profile
            )
        else:
            self.broadcaster.sendTransform([self.static_tf_summit, self.static_tf_ranger])
        
    def handle_odom(self, msg):
        """Callback function for vehicle_odometry topic subscriber, used to create a dynamic tf between world (fixed frame in Rviz) and drone's base frame"""
        self.tf_px4 = TransformStamped()
        self.tf_px4.header.stamp = self.get_clock().now().to_msg()
        self.tf_px4.header.frame_id  = 'world'
        self.tf_px4.child_frame_id = 'base_link'
        # Estimated position is in PX4's coordintae system and therefore needs to be transformed to Rviz's 
        self.tf_px4.transform.translation.x = float(msg.position[1])
        self.tf_px4.transform.translation.y = float(msg.position[0])
        self.tf_px4.transform.translation.z = -float(msg.position[2]) + 0.24

        # Using Euler rotations around x & z axis (as recommended by PX4's documentation) does not work for some reasons
        # There is a permutation betwwen rotations around x & z axis that I could not explain
        # Switching the x & z quaternions did the trick, but I'm not really sure about the logic behind
        q_rviz =  (Rotation.from_quat(np.array([-msg.q[2], -msg.q[1], msg.q[0], msg.q[3]])) * Rotation.from_euler('z', np.deg2rad(-100))).as_quat()
        self.tf_px4.transform.rotation.x = float(q_rviz[0])
        self.tf_px4.transform.rotation.y = float(q_rviz[1])
        self.tf_px4.transform.rotation.z = float(q_rviz[2])
        self.tf_px4.transform.rotation.w = float(q_rviz[3])
        
        self.broadcaster.sendTransform([self.static_tf_summit, self.static_tf_ranger, self.tf_px4])

def main():
    rclpy.init()
    node = TfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
