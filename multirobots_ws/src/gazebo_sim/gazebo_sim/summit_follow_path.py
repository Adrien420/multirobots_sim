import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from scipy.spatial.transform import Rotation
import numpy as np
import yaml, math

class SummitFollowPath(Node):
    def __init__(self):
        super().__init__('summit_follow_path')

        self.cmd_pub = self.create_publisher(TwistStamped, '/summit_xl_1/robotnik_base_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/summit_xl_1/robotnik_base_controller/odom', self.odom_callback, 10)

        self.path = self.load_path_yaml('/home/multirobots/multirobots_ws/install/gazebo_sim/share/gazebo_sim/config/summit_path.yaml', 'traj1')
        self.current_odom_pose = None
        self.current_target_pose = 0
        self.timer = self.create_timer(0.05, self.follow_path)  # 20 Hz

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

    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose.pose

    def follow_path(self):
        if not self.path or self.current_odom_pose==None:
            return

        # Prend la prochaine cible
        target = self.path.poses[self.current_target_pose].pose
        # Calcul simple de distance 2D
        dx = target.position.x - self.current_odom_pose.position.x
        dy = target.position.y - self.current_odom_pose.position.y
        distance = np.hypot(dx, dy)

        # Contrôle proportionnel simple
        k_linear = 2.0
        k_angular = 2.0

        angle_to_goal = np.arctan2(dy, dx)

        yaw = self.get_yaw_from_quaternion(self.current_odom_pose.orientation)
        target_angle = self.get_yaw_from_quaternion(target.orientation)
        angle_error = self.normalize_angle(angle_to_goal - yaw)
        angle_target_error = self.normalize_angle(target_angle - yaw)
        print(f'target : {target_angle} / angle_error : {angle_error} / angle_target_error : {angle_target_error} / distance : {distance}')

        if distance < 0.01 and abs(angle_target_error) < 0.035:
            if self.current_target_pose < len(self.path.poses)-1:
                self.current_target_pose += 1
                self.get_logger().info(f"Reached a waypoint, {len(self.path.poses)-self.current_target_pose} remaining.")
                self.get_logger().info(f"x : {self.current_odom_pose.position.x}, y : {self.current_odom_pose.position.y}, z (rot) : {yaw}")
            else:
                self.get_logger().info(f"Reached endpoint")
                self.get_logger().info(f"x : {self.current_odom_pose.position.x}, y : {self.current_odom_pose.position.y}, z (rot) : {yaw}")
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
        cmd.twist.linear.x = min(0.5, k_linear * distance)
        if distance < 0.01:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = k_angular * angle_target_error
        else:
            cmd.twist.angular.z = k_angular * angle_error

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
    node = SummitFollowPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
