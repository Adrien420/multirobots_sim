#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')

        self.broadcaster = StaticTransformBroadcaster(self)

        # Create static tf between world and summit's base frame, without any translation or rotation
        static_tf_summit = TransformStamped()
        static_tf_summit.header.stamp = self.get_clock().now().to_msg()
        static_tf_summit.header.frame_id = 'world'
        static_tf_summit.child_frame_id = 'summit_xl_1/summit_xl_odom'

        static_tf_summit.transform.translation.x = 0.0
        static_tf_summit.transform.translation.y = 0.0
        static_tf_summit.transform.translation.z = 0.0

        static_tf_summit.transform.rotation.x = 0.0
        static_tf_summit.transform.rotation.y = 0.0
        static_tf_summit.transform.rotation.z = 0.0
        static_tf_summit.transform.rotation.w = 1.0

        # Create static tf between world and px4 drone's base frame, without any rotation (and just 1translation so that the robots are spaed the same way that in gz)
        static_tf_px4 = TransformStamped()
        static_tf_px4.header.stamp = self.get_clock().now().to_msg()
        static_tf_px4.header.frame_id = 'world'
        static_tf_px4.child_frame_id = 'base_link'

        static_tf_px4.transform.translation.x = 0.0
        static_tf_px4.transform.translation.y = -1.0
        static_tf_px4.transform.translation.z = 0.0

        static_tf_px4.transform.rotation.x = 0.0
        static_tf_px4.transform.rotation.y = 0.0
        static_tf_px4.transform.rotation.z = 0.0
        static_tf_px4.transform.rotation.w = 1.0

        # Envoyer la transform statique
        self.broadcaster.sendTransform([static_tf_summit, static_tf_px4])

def main():
    rclpy.init()
    node = StaticTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
