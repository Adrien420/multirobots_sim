import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from gps_msgs.msg import GPSFix
from scipy.spatial.transform import Rotation
import numpy as np
import yaml, math

class SummitFollowPath(Node):
    def __init__(self):
        super().__init__('summit_follow_path')

        # Parameters
        self.declare_parameter('summit_id', 1)
        self.summit_prefix = '/summit_xl_' + str(self.get_parameter('summit_id').value)

        self.declare_parameter('path_name', 'path1')
        self.declare_parameter('velocity', 2.0)

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, self.summit_prefix + '/robotnik_base_controller/cmd_vel', 10)

        # Subscribers
        # Odometry was previously used, but was too far from ground truth (especially because of some physics issues with Gazebo's physics engine)
        #self.create_subscription(Odometry, self.summit_prefix + '/robotnik_base_controller/odom', self.odom_callback, 10)
        self.create_subscription(Imu, self.summit_prefix + '/imu_data', self.imu_callback, 10)
        self.create_subscription(GPSFix, self.summit_prefix + '/navsat_data', self.navsat_callback, 10)

        # Load the path to follow
        self.path = self.load_path_yaml('/home/multirobots/multirobots_ws/install/gazebo_sim/share/gazebo_sim/config/summit_path.yaml', self.get_parameter('path_name').value)
        #self.current_odom_pose = None
        
        # Initialize variables
        self.current_target_pose = 0

        self.current_imu = None
        self.reaching_goal_orientation = True
        self.reaching_target_orientation = False

        self.current_navsat = None
        self.firts_navsat_msg = True
        self.longitude_init = 0
        self.latitude_init = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.3, self.follow_path)

    def load_path_yaml(self, yaml_file, path_name):
        """Load the yaml configuration file containing paths for Summit XL & get positions of the chosen path"""
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        # Positions of the path
        poses = data[path_name]['poses']
        # Create a Path msg for Rviz visualization (Interfacing with Rviz not implemented yet)
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        for pose_ in poses:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = pose_[0] 
            pose.pose.position.y = pose_[1] 
            yaw = pose_[2]
            # Transform the desired orientation in quatrenions
            q = Rotation.from_euler('z', np.deg2rad(yaw)).as_quat()
            pose.pose.orientation.x = q[0] 
            pose.pose.orientation.y = q[1] 
            pose.pose.orientation.z = q[2] 
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        return path

    # def odom_callback(self, msg):
    #     self.current_odom_pose = msg.pose.pose

    def imu_callback(self, msg):
        """Callback function for IMU data"""
        self.current_imu = msg

    def navsat_callback(self, msg):
        """Callback function for NavSat data"""
        self.current_navsat = msg

    def follow_path(self):
        """Follow positions of the selected path"""
        # Exit the function if there is no data available in the topics of IMU or NavSat sensors
        if not self.path or self.current_navsat==None or self.current_imu==None:
            return
        
        # Register the firt latitude & longitude given by the NavSat sensor, needed to compute position in meters according to x & y axis
        if self.firts_navsat_msg:
            self.longitude_init = self.current_navsat.longitude
            self.latitude_init = self.current_navsat.latitude
            self.firts_navsat_msg = False

        target = self.path.poses[self.current_target_pose].pose

        # Odometry (orientation errors too important to be exploitable, because of physics of the simulation)
        # dx = target.position.x - self.current_odom_pose.position.x
        # dy = target.position.y - self.current_odom_pose.position.y

        # Earth's radius in meters
        R=6378137

        # Transform longitude & latitude in x & y using Mercator projection
        dlat = R * np.log(np.tan(np.pi/4 + np.radians(self.current_navsat.latitude)/2))
        dlon = R * np.radians((self.current_navsat.longitude - self.longitude_init))

        # Computation of the error in translation using NavSat sensor data
        dx = target.position.x - dlon
        dy = target.position.y - dlat
        distance = np.sqrt(dx**2 + dy**2)

        # Coefficients for linear & angular velocities
        k_linear = 1.0
        k_angular = 1.0

        # Compute the angle to face goal position only if far from it
        if distance > 0.3:
            angle_to_goal = np.arctan2(dy, dx)
        else:
            angle_to_goal = 0.0

        #yaw = self.get_yaw_from_quaternion(self.current_odom_pose.orientation)
        # Get yaw rotation from IMU's data
        yaw = self.get_yaw_from_quaternion(self.current_imu.orientation)
        target_angle = self.get_yaw_from_quaternion(target.orientation)
        goal_angle_error = self.normalize_angle(angle_to_goal - yaw)
        target_angle_error = self.normalize_angle(target_angle - yaw)
        
        # Print data about orientations
        #print(f'current_angle : {yaw} / target_angle : {target_angle} / target_angle_error : {target_angle_error} / angle_to_goal : {angle_to_goal} / goal_angle_error : {goal_angle_error}')
        # Print data about position
        #print(f'current_x : {dlon} / current_y : {dlat} / dx : {dx} / dy : {dy} / distance : {distance}')

        # If the error (in translation & rotation) is small enough, we move on to the next position
        if abs(target_angle_error) < 0.035 and self.reaching_target_orientation:
            if self.current_target_pose < len(self.path.poses)-1:
                self.current_target_pose += 1
                self.reaching_target_orientation = False
                self.reaching_goal_orientation = True
                self.get_logger().info(f"Reached a waypoint, {len(self.path.poses)-self.current_target_pose} remaining.")
                self.get_logger().info(f"x : {dlon}, y : {dlat}, z (rot) : {yaw}")
            else:
                self.get_logger().info(f"Reached endpoint")
                self.get_logger().info(f"x : {dlon}, y : {dlat}, z (rot) : {yaw}")
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = ''
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                raise SystemExit
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ''

        # When goal orientation is reached (step 1, see below), move on to step 2
        if abs(goal_angle_error) < 0.035 and self.reaching_goal_orientation:
            self.reaching_goal_orientation = False
        # If goal angle error becomes too important again in step 2, because of the terrain, we go back to step 1 to avoid too much derivation (causing troubles at the arrival) 
        elif abs(goal_angle_error) > 0.174 and not self.reaching_target_orientation and distance > 0.3:
            self.reaching_goal_orientation = True

        # Step 1 : The robot rotate to face goal position
        if self.reaching_goal_orientation:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = k_angular * goal_angle_error
            #print("Step 1")
        # Step 3 : Once the position targetted is reached, the robot starts rotating on the spot to reach the targetted orientation
        # (To avoid switching between step 3 and step 2 for ever, we don't go back to step 2 even if distance is increasing because of slipping)
        elif distance < 0.3 or self.reaching_target_orientation:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = k_angular * target_angle_error
            self.reaching_target_orientation = True
            #print("Step 3")
        # Step 2 : The robot runs (almost in straight line) until reaching the goal position, while still adjusting its orientation, to avoid derivation
        # (If step 1 isn't done, the robot can pass next to the goal position because of its orientation not correctly adjusted yet, causing it to make circles around the goal position, trying to reach it)
        else:
            cmd.twist.linear.x = min(float(self.get_parameter('velocity').value), k_linear * distance)
            cmd.twist.angular.z = k_angular * goal_angle_error
            #print("Step 2")
            

        self.cmd_pub.publish(cmd)

    def get_yaw_from_quaternion(self, q):
        """Get the yaw rotation from the orientation in quaterions"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize the angle to remain in the [-pi;pi] range"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

def main():
    rclpy.init()
    node = SummitFollowPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
