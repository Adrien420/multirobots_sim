import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

keys_manual = """
Keys to control the ranger mini :
---------------------------
Moving around:
        z   
    q   s   d

z/s : translate forward / backward
q/d : rotate to the left / to the right
    
CTRL-C to quit
"""

class RangerMiniTeleop(Node) :
    def __init__(self):
        super().__init__('ranger_mini_teleop')
        
        # Parameters
        # self.declare_parameter('summit_id', 1)
        # self.summit_name = '/summit_xl_' + str(self.get_parameter('summit_id').value)
    
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Initialize variables
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 5.0
        self.rotationSpeed = 2.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
    
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
        msg = Twist()
        
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = ''
        
        key = self.get_key()
        # Movements
        # Translation according to x (forward/backward)
        if key == 'z':
            msg.linear.x = 1.0 * self.speed
        elif key == 's':
            msg.linear.x = -1.0 * self.speed
        # Rotation according to z (left/right)
        elif key == 'q':
            msg.angular.z = 1.0 * self.rotationSpeed
        elif key == 'd':
            msg.angular.z = -1.0 * self.rotationSpeed
        # Ctrl+C
        elif key == '\x03':
            raise KeyboardInterrupt
    
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_velocity()
        
def main():
    rclpy.init()
    ranger_mini_teleop = RangerMiniTeleop()
    print(keys_manual)
    
    try:
        rclpy.spin(ranger_mini_teleop)
    except KeyboardInterrupt:
        print("CTRL+C detected : process stopped")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ranger_mini_teleop.settings)
        ranger_mini_teleop.destroy_node()
        rclpy.shutdown()        
        
if __name__ == '__main__':
    main()