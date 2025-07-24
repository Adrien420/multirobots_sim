import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition

import sys
import select
import termios
import tty

keys_manual = """
Keys to control the x500 :
---------------------------
Moving around:
        z
    q   s   d

z/s : translate forward / backward
q/d : translate to the left / to the right

                  arrow_up
    arrow_left   arrow_down   arrow_right
    
arrow_up/arrow_down : translate upward / downward
arrow_left/arrow_right : rotate to the left / to the right
    
CTRL-C to quit
"""

class Px4Teleop(Node) :
    def __init__(self):
        super().__init__('px4_teleop')
        
        # Parameters
        self.declare_parameter('drone_id', 1)

        # Configure QoS profile for publishing and subscribing to PX4's topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        prefix_path = '/px4_' + str(self.get_parameter('drone_id').value) + '/fmu/'
        print(prefix_path + 'in/offboard_control_mode')
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, prefix_path + 'in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, prefix_path + 'in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, prefix_path + 'in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, prefix_path + 'out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, prefix_path + 'out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.reboot_cmd_sent = False
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.settings = termios.tcgetattr(sys.stdin)

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def get_key(self):
        """Detect the key currently pressed by reading in the stdin"""
        tty.setraw(sys.stdin.fileno())
        # Select stdin to read
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1) # 1 char
            if key == '\x1b': # Arrow -> 2 chars
                key += sys.stdin.read(2)
        else:
            key = ''
                
        return key

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber"""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber"""
        self.vehicle_status = vehicle_status
        
    def reboot(self):
        """Send a reboot command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            param1=1.0
        )
        self.get_logger().info("Reboot command sent")
        self.reboot_cmd_sent = True

    def arm(self):
        """Send an arm command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity(self):
        """Publish the trajectory setpoint"""
        msg = TrajectorySetpoint()
        
        key = self.get_key()
        # Movements
        # Translation according to y (left/right)
        if key == 'q':
            msg.velocity[0] = 1.0 
        elif key == 'd':
            msg.velocity[0] = -1.0
        # Translation according to x (forward/backward)
        elif key == 'z':
            msg.velocity[1] = 1.0
        elif key == 's':
            msg.velocity[1] = -1.0
        # Translation according to z (up/down)
        elif key == '\x1b[A':
            msg.velocity[2] = -1.0
        elif key == '\x1b[B':
            msg.velocity[2] = 1.0
        # Rotation around z (left/right) 
        elif key == '\x1b[D':
            msg.yawspeed = -1.0
        elif key == '\x1b[C':
            msg.yawspeed = 1.0
        # Ctrl+C
        elif key == '\x03':
            raise KeyboardInterrupt
        
        # Horizontal position
        # Disable position controller
        msg.position[0] = float("nan")
        msg.position[1] = float("nan")
        msg.yaw = float("nan")

        # Vertical position
        if key == '\x1b[A' or key == '\x1b[B':
            msg.position[2] = float("nan") # Disable position controller for altitude
        else: # If no velocity commands are sent to control the altitude, let the drone hover at its current height
            msg.position[2] = self.vehicle_local_position.z
    
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        # Drone targetted : drone's id + 1
        msg.target_system = self.get_parameter('drone_id').value + 1 
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer"""
        # Check if the drone is ready to be armed
        if not self.vehicle_status.pre_flight_checks_pass:
            if self.vehicle_status.latest_disarming_reason == 6 and not self.reboot_cmd_sent: # Failsafe
                self.reboot()
                return
            else: # Initialization
                return
        
        self.publish_offboard_control_heartbeat_signal()

        # Switch to offboard mode & arm the drone after spending enough signals to the drone
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            print(keys_manual)
            
        # Switch back to offboard & arm the drone again (for instance, if the drone has switched mode after entering failsafe mode)
        if self.offboard_setpoint_counter > 15 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.engage_offboard_mode()
            self.arm()

        # In offboard mode, send the targetted velocity
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_velocity()

        self.offboard_setpoint_counter += 1
        
def main():
    rclpy.init()
    px4_teleop = Px4Teleop()
    print("Node initialized : Waiting for px4 to be initialized...")
    
    try:
        rclpy.spin(px4_teleop)
    except KeyboardInterrupt:
        print("CTRL+C detected : process stopped")
    finally:
        px4_teleop.land()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, px4_teleop.settings)
        px4_teleop.destroy_node()
        rclpy.shutdown()        
        
if __name__ == '__main__':
    main()