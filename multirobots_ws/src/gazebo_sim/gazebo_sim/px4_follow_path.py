import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation
import numpy as np
import yaml, math

class Px4FollowPath(Node) :
    def __init__(self):
        super().__init__('px4_follow_path')
        
        # Parameters
        self.declare_parameter('drone_id', 1)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        prefix_path = '/px4_' + str(self.get_parameter('drone_id').value) + '/fmu/'
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

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.path = self.load_path_yaml('/home/multirobots/multirobots_ws/install/gazebo_sim/share/gazebo_sim/config/px4_path.yaml', 'traj1')
        self.current_target_pose = 0

    def load_path_yaml(self, yaml_file, path_name):
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        poses = data[path_name]['poses']
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        for pose_ in poses:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = pose_[0] 
            pose.pose.position.y = pose_[1] 
            pose.pose.position.z = pose_[2]
            yaw = pose_[3]
            q = Rotation.from_euler('z', np.deg2rad(yaw)).as_quat()
            pose.pose.orientation.x = q[0] 
            pose.pose.orientation.y = q[1] 
            pose.pose.orientation.z = q[2] 
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        return path

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        
    def reboot(self):
        """Send a reboot command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            param1=1.0
        )
        self.get_logger().info("Reboot command sent")
        self.reboot_cmd_sent = True

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()

        target_pose = self.path.poses[self.current_target_pose].pose
        
        msg.position[0] = target_pose.position.y
        msg.position[1] = target_pose.position.x
        msg.position[2] = -target_pose.position.z
        msg.yaw = self.get_yaw_from_quaternion(target_pose.orientation) + np.pi/2

        dx = target_pose.position.y - self.vehicle_local_position.x
        dy = target_pose.position.x - self.vehicle_local_position.y
        dz = target_pose.position.z + self.vehicle_local_position.z
        distance = math.sqrt(dx**2+dy**2+dz**2)

        target_angle = self.get_yaw_from_quaternion(target_pose.orientation)
        angle_target_error = self.normalize_angle(target_angle + np.pi/2 - self.vehicle_local_position.heading)
        print(f'target : {target_angle} / angle_target_error : {angle_target_error} / distance : {distance} / {dx} / {dy} / {dz}')

        if distance < 0.05 and abs(angle_target_error) < 0.035:
            if self.current_target_pose < len(self.path.poses)-1:
                self.current_target_pose += 1
                self.get_logger().info(f"Reached a waypoint, {len(self.path.poses)-self.current_target_pose} remaining.")
                self.get_logger().info(f"x : {self.vehicle_local_position.y}, y : {self.vehicle_local_position.x}, z (alt) : {-self.vehicle_local_position.z} z (rot) : {self.vehicle_local_position.heading}")
            else:
                self.get_logger().info(f"Reached endpoint")
                self.get_logger().info(f"x : {self.vehicle_local_position.y}, y : {self.vehicle_local_position.x}, z (alt) : {-self.vehicle_local_position.z} z (rot) : {self.vehicle_local_position.heading}")
                raise SystemExit
    
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.get_parameter('drone_id').value + 1 # Drone targetted : drone's id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        if not self.vehicle_status.pre_flight_checks_pass:
            if self.vehicle_status.latest_disarming_reason == 6 and not self.reboot_cmd_sent: # Failsafe
                self.reboot()
                return
            else: # Initialization
                return
        
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            
        if self.offboard_setpoint_counter > 15 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_velocity()

        self.offboard_setpoint_counter += 1
        
def main():
    rclpy.init()
    px4_teleop = Px4FollowPath()
    print("Node initialized : Waiting for px4 to be initialized...")
    rclpy.spin(px4_teleop)

    px4_teleop.land()
    px4_teleop.destroy_node()
    rclpy.shutdown()        
        
if __name__ == '__main__':
    main()