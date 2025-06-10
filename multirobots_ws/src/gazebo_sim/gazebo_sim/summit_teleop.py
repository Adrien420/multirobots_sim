import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

import sys
import select
import termios
import tty

keys_manual = """
Keys to control the summit xl :
---------------------------
Moving around:
    a   z   e
    q   s   d

z/s : translate forward / backward
q/d : rotate to the left / to the right
a/e : combination of z & q / z & d
    
CTRL-C to quit
"""

class SummitTeleop(Node) :
    def __init__(self):
        super().__init__('summit_teleop')
        
        # Parameters
        self.declare_parameter('summit_id', 1)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        # self.sub = self.create_subscription(LaserScan, '/world/forest/model/summit_xl/link/summit_xl_base_footprint/sensor/summit_xl_front_laser_sensor/scan', self.callback, qos)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/robotnik_base_controller/cmd_vel', qos_profile)

        # Initialize variables
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 2.0
        self.rotationSpeed = 2.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    # def callback(self, msg):
    #     self.get_logger().info(f"Received PointCloud2 message with width: {msg.intensities}")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b': # Arrow -> 2 chars
                key += sys.stdin.read(2)
        else:
            key = ''
                
        return key

    def publish_velocity(self):
        """Publish velocity messages."""
        msg = TwistStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        key = self.get_key()
        # Movements
        # Translation according to x (forward/backward)
        if key == 'z':
            msg.twist.linear.x = 1.0 * self.speed
        elif key == 's':
            msg.twist.linear.x = -1.0 * self.speed
        # Rotation according to z (left/right)
        elif key == 'q':
            msg.twist.angular.z = 1.0 * self.rotationSpeed
        elif key == 'd':
            msg.twist.angular.z = -1.0 * self.rotationSpeed
        # Combination of translation & rotation
        elif key == 'a':
            msg.twist.angular.z = 1.0 * self.rotationSpeed
            msg.twist.linear.x = 1.0 * self.speed
        elif key == 'e':
            msg.twist.angular.z = -1.0 * self.rotationSpeed
            msg.twist.linear.x = 1.0 * self.speed
        # Ctrl+C
        elif key == '\x03':
            raise KeyboardInterrupt
    
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_velocity()
        
def main():
    rclpy.init()
    summit_teleop = SummitTeleop()
    print(keys_manual)
    
    try:
        rclpy.spin(summit_teleop)
    except KeyboardInterrupt:
        print("CTRL+C detected : process stopped")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, summit_teleop.settings)
        summit_teleop.destroy_node()
        rclpy.shutdown()        
        
if __name__ == '__main__':
    main()