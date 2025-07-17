#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

import numpy as np

class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter('nb_drones', 0)

        self.broadcaster = StaticTransformBroadcaster(self)

        # Create static tf between world and summit's base frame, without any translation or rotation
        self.static_tf_summit = TransformStamped()
        self.static_tf_summit.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_summit.header.frame_id = 'world'
        self.static_tf_summit.child_frame_id = 'summit_xl_1/summit_xl_odom'

        self.static_tf_summit.transform.translation.x = 0.0
        self.static_tf_summit.transform.translation.y = 1.0
        self.static_tf_summit.transform.translation.z = 0.0

        self.static_tf_summit.transform.rotation.x = 0.0
        self.static_tf_summit.transform.rotation.y = 0.0
        self.static_tf_summit.transform.rotation.z = 0.0
        self.static_tf_summit.transform.rotation.w = 1.0
        
        if self.get_parameter('nb_drones').value > 0:
            self.subscription = self.create_subscription(
                VehicleOdometry,
                '/px4_1/fmu/out/vehicle_odometry',
                self.handle_odom,
                qos_profile
            )
        else:
            self.broadcaster.sendTransform(self.static_tf_summit)
        
    def handle_odom(self, msg):
        self.tf_px4 = TransformStamped()
        self.tf_px4.header.stamp = self.get_clock().now().to_msg()
        self.tf_px4.header.frame_id  = 'world'
        self.tf_px4.child_frame_id = 'base_link'
        self.tf_px4.transform.translation.x = float(msg.position[1])
        self.tf_px4.transform.translation.y = float(msg.position[0])
        self.tf_px4.transform.translation.z = -float(msg.position[2]) + 0.24

        q_rviz =  (Rotation.from_quat(np.array([-msg.q[2], -msg.q[1], msg.q[0], msg.q[3]])) * Rotation.from_euler('z', np.deg2rad(-100))).as_quat()
        #q_rviz =  (Rotation.from_quat(np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])) * Rotation.from_euler('z', np.deg2rad(90)) * Rotation.from_euler('x', np.deg2rad(180))).as_quat()
        #print((Rotation.from_quat(np.array([msg.q[2], msg.q[1], msg.q[0], msg.q[3]])) * Rotation.from_euler('z', np.deg2rad(-90))).as_euler('xyz', degrees=True))
        self.tf_px4.transform.rotation.x = float(q_rviz[0])
        self.tf_px4.transform.rotation.y = float(q_rviz[1])
        self.tf_px4.transform.rotation.z = float(q_rviz[2])
        self.tf_px4.transform.rotation.w = float(q_rviz[3])
        
        # Envoyer les tfs
        self.broadcaster.sendTransform([self.static_tf_summit, self.tf_px4])

def main():
    rclpy.init()
    node = StaticTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
