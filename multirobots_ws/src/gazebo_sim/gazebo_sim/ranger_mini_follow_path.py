import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from gps_msgs.msg import GPSFix
from scipy.spatial.transform import Rotation
import numpy as np
import yaml, math

class RangerFollowPath(Node):
    def __init__(self):
        super().__init__('ranger_follow_path')

        self.declare_parameter('ranger_id', 1)
        self.ranger_prefix = '/ranger_mini_' + str(self.get_parameter('ranger_id').value)

        self.declare_parameter('path_name', 'square')
        self.declare_parameter('velocity', 2.0)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.ranger_prefix + '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Imu, self.ranger_prefix + '/imu_data', self.imu_callback, 10)
        self.create_subscription(GPSFix, self.ranger_prefix + '/navsat_data', self.navsat_callback, 10)

        self.path = self.load_path_yaml('/home/multirobots/multirobots_ws/install/gazebo_sim/share/gazebo_sim/config/ranger_path.yaml', self.get_parameter('path_name').value)
        self.current_target_pose = 0

        self.current_imu = None
        self.reaching_goal_orientation = True
        self.reaching_target_orientation = False

        self.current_navsat = None
        self.firts_navsat_msg = True
        self.longitude_init = 0
        self.latitude_init = 0

        self.timer = self.create_timer(0.3, self.follow_path)  # 20 Hz

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
            yaw = pose_[2]
            q = Rotation.from_euler('z', np.deg2rad(yaw)).as_quat()
            pose.pose.orientation.x = q[0] 
            pose.pose.orientation.y = q[1] 
            pose.pose.orientation.z = q[2] 
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        return path

    def imu_callback(self, msg):
        self.current_imu = msg

    def navsat_callback(self, msg):
        self.current_navsat = msg

    def follow_path(self):
        if not self.path or self.current_navsat==None or self.current_imu==None:
            return
        
        if self.firts_navsat_msg:
            self.longitude_init = self.current_navsat.longitude
            self.latitude_init = self.current_navsat.latitude
            self.firts_navsat_msg = False

        # Prend la prochaine cible
        target = self.path.poses[self.current_target_pose].pose

        # NavSat
        R=6378137

        # Mercator projection
        dlat = R * np.log(np.tan(np.pi/4 + np.radians(self.current_navsat.latitude)/2))
        dlon = R * np.radians((self.current_navsat.longitude - self.longitude_init))

        dx = target.position.x - dlon
        dy = target.position.y - dlat
        distance = np.sqrt(dx**2 + dy**2)

        # Contrôle proportionnel simple
        k_linear = 1.0
        k_angular = 1.0

        if distance > 0.3:
            angle_to_goal = np.arctan2(dy, dx)
        else:
            angle_to_goal = 0.0

        #yaw = self.get_yaw_from_quaternion(self.current_odom_pose.orientation)
        yaw = self.get_yaw_from_quaternion(self.current_imu.orientation)
        target_angle = self.get_yaw_from_quaternion(target.orientation)
        goal_angle_error = self.normalize_angle(angle_to_goal - yaw)
        target_angle_error = self.normalize_angle(target_angle - yaw)
        
        # Print data about orientations
        #print(f'current_angle : {yaw} / target_angle : {target_angle} / target_angle_error : {target_angle_error} / angle_to_goal : {angle_to_goal} / goal_angle_error : {goal_angle_error}')
        # Print data about position
        #print(f'current_x : {dlon} / current_y : {dlat} / dx : {dx} / dy : {dy} / distance : {distance}')

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
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                raise SystemExit
            return

        cmd = Twist()

        # When goal orientation is reached (step 1), move on to step 2
        if abs(goal_angle_error) < 0.035 and self.reaching_goal_orientation:
            self.reaching_goal_orientation = False
        # If goal angle error becomes too important again in step 2, because of the terrain, we go back to step 1 to avoid too much derivation (causing troubles at the arrival) 
        elif abs(goal_angle_error) > 0.174 and not self.reaching_target_orientation and distance > 0.3:
            self.reaching_goal_orientation = True

        # Step 1 : The robot rotate to face goal position
        if self.reaching_goal_orientation:
            cmd.linear.x = 0.0
            cmd.angular.z = k_angular * goal_angle_error

        # Step 3 : Once the position targetted is reached, the robot starts rotating on the spot to reach the targetted orientation
        # (To avoid switching between step 3 and step 2 for ever, we don't go back to step 2 even if distance is increasing because of slipping)
        elif distance < 0.3 or self.reaching_target_orientation:
            cmd.linear.x = 0.0
            cmd.angular.z = k_angular * target_angle_error
            self.reaching_target_orientation = True

        # Step 2 : The robot runs (almost in straight line) until reaching the goal position, while still adjusting its orientation, to avoid derivation
        # (If step 1 isn't done, the robot can pass next to the goal position because of its orientation not correctly adjusted yet, causing it to make circles around the goal position, trying to reach it)
        else:
            cmd.linear.x = min(float(self.get_parameter('velocity').value), k_linear * distance)
            cmd.angular.z = k_angular * goal_angle_error
            

        self.cmd_pub.publish(cmd)

    def get_yaw_from_quaternion(self, q):
        # ROS q = [x, y, z, w]
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

def main():
    rclpy.init()
    node = RangerFollowPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
