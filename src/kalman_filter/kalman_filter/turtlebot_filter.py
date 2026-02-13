import rclpy
import numpy as np
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path, Odometry
from kalman import KalmanFilter
from kalman_prams import kf_prams

PUBLISHER_FREQUENCY_HZ = 20
class Localizer(Node): 
    def __init__(self):
        super().__init__('localizer_node')
        
        self.delta_t = 1/PUBLISHER_FREQUENCY_HZ

        self.F, self.B, self.H, self.Q,self.R, self.x0, self.P0 = kf_prams(self.delta_t)
        self.kalman_filter = KalmanFilter(self.F, self.H, self.Q, self.R,self.B, self.x0, self.P0)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.joint_state_subscription = self.create_subscription(JointState,'/joint_states', self.joint_state_callback, qos_profile)
        self.imu_subscription = self.create_subscription(Imu,'/imu', self.imu_callback, qos_profile)
        self.cmd_vel_subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmd_vel_callback, qos_profile)

        self.kf_path_publisher = self.create_publisher(Path, 'localization_node/kf/path', 10)
        self.ekf_path_publisher = self.create_publisher(Path, 'localization_node/ekf/path', 10)
        self.ukf_path_publisher = self.create_publisher(Path, 'localization_node/ukf/path', 10)

        self.kf_odom_publisher = self.create_publisher(Odometry, 'localization_node/kf/odometry', 10)
        self.ekf_odom_publisher = self.create_publisher(Odometry, 'localization_node/ekf/odometry', 10)
        self.ukf_odom_publisher = self.create_publisher(Odometry, 'localization_node/ukf/odometry', 10)

    def joint_state_callback(self, msg):
        # check what these actually are
        self.js_x = msg.position[0]
        self.js_y = msg.position[1]
    
    def imu_callback(self, msg):
        self.imu_w = msg.angular_velocity.z
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y

    def cmd_vel_callback(self, msg):
        self.cmd_vel_x = msg.twist.linear.x
        self.cmd_vel_y = msg.twist.linear.y
        self.cmd_vel_w = msg.twist.angular.z

    def kf_predict(self):
        theta = self.kalman_filter.x[6,0]
        world_x_vel = (self.cmd_vel_x*m.cos(theta)) - (self.cmd_vel_y*m.sin(theta))
        world_y_vel = (self.cmd_vel_x*m.sin(theta)) + (self.cmd_vel_y*m.cos(theta))
        world_w = self.cmd_vel_w
        u = np.array([[0],
                      [world_x_vel],
                      [0],
                      [0],
                      [world_y_vel],
                      [0],
                      [0],
                      [world_w],
                      [0]])
        return self.kalman_filter.predict(u)

    def kf_update(self):
        theta = self.kalman_filter.x[6,0]
        world_x_acc = (self.cmd_vel_x*m.cos(theta)) - (self.cmd_vel_y*m.sin(theta))
        world_y_acc = (self.cmd_vel_x*m.sin(theta)) + (self.cmd_vel_y*m.cos(theta))
        world_t = 
        world_w = self.cmd_vel_w
        u = np.array([[0],
                      [0],
                      [0],
                      [0],
                      [0],
                      [0],
                      [0],
                      [0],
                      [0]])
        return self.kalman_filter.predict(u)



def main(args=None):
    rclpy.init(args=args)
    localizer_node = Localizer()
    rclpy.spin(localizer_node)
    localizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


