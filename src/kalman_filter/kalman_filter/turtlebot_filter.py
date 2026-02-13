import rclpy
import numpy as np
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from kalman import KalmanFilter
from kalman_prams import Kf_prams
from rclpy.clock import Clock
from euler_quaternion import convertEulerToQuaternion

PUBLISHER_FREQUENCY_HZ = 20
ROBOT_WHEELBASE = 1
ROBOT_WHEEL_RADIUS = 1
class Localizer(Node): 
    def __init__(self):
        super().__init__('localizer_node')
        
        self.delta_t = 1/PUBLISHER_FREQUENCY_HZ
        self.timer = self.create_timer(self.delta_t, self.timer_callback)

        kf = Kf_prams(self.delta_t)
        self.kalman_filter = KalmanFilter(kf.F, kf.H, kf.Q, kf.R, kf.B, kf.x0, kf.P0)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.joint_state_subscription = self.create_subscription(JointState,'/joint_states', self.joint_state_callback, qos_profile)
        self.FRESH_JOINT_STATE = False
        self.imu_subscription = self.create_subscription(Imu,'/imu', self.imu_callback, qos_profile)
        self.FRESH_IMU = False
        self.cmd_vel_subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.FRESH_CMD_VEL = False    

        self.kf_path_publisher = self.create_publisher(Path, 'localization_node/kf/path', 10)
        self.kf_path_arr = []
        self.ekf_path_publisher = self.create_publisher(Path, 'localization_node/ekf/path', 10)
        self.ukf_path_publisher = self.create_publisher(Path, 'localization_node/ukf/path', 10)

        self.kf_odom_publisher = self.create_publisher(Odometry, 'localization_node/kf/odometry', 10)
        self.ekf_odom_publisher = self.create_publisher(Odometry, 'localization_node/ekf/odometry', 10)
        self.ukf_odom_publisher = self.create_publisher(Odometry, 'localization_node/ukf/odometry', 10)
        

    def joint_state_callback(self, msg):
        # check what these actually are
        self.js_right_wheel = msg.position[0]
        self.js_left_wheel = msg.position[1]
        self.FRESH_JOINT_STATE = True
    
    def imu_callback(self, msg):
        self.imu_w = msg.angular_velocity.z
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.FRESH_IMU = True

    def cmd_vel_callback(self, msg):
        self.cmd_vel_x = msg.twist.linear.x
        self.cmd_vel_y = msg.twist.linear.y
        self.cmd_vel_w = msg.twist.angular.z
        self.FRESH_CMD_VEL = True

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
        world_x_acc = (self.imu_ax*m.cos(theta)) - (self.imu_ay*m.sin(theta))
        world_y_acc = (self.imu_ax*m.sin(theta)) + (self.imu_ay*m.cos(theta))
        world_t = (2*m.pi*ROBOT_WHEEL_RADIUS*(self.js_right_wheel-self.js_left_wheel))/ROBOT_WHEELBASE
        world_w = self.cmd_vel_w
        z = np.array([[0],
                      [0],
                      [world_x_acc],
                      [0],
                      [0],
                      [world_y_acc], 
                      [world_t],
                      [world_w],
                      [0]])
        return self.kalman_filter.update(z)
    
    def timer_callback(self):
        if(self.FRESH_JOINT_STATE and self.FRESH_IMU):
            
            x_prediction = self.kf_predict()
            x_update, residual = self.kf_update()
            self.FRESH_JOINT_STATE = False
            self.FRESH_IMU = False
            self.FRESH_CMD_VEL = False
            
            #generate pose message
            current_pose = PoseStamped()
            current_pose.header.stamp = Clock().now().to_msg()
            current_pose.header.frame_id = 'odom'
            current_pose.pose.position.x = x_update.x[0,0]
            current_pose.pose.position.y = x_update.x[3,0]

            #get orientation quaternion
            w,x,y,z = convertEulerToQuaternion(0,0,x_update.x[6,0])
            current_pose.pose.orientation.w = w
            current_pose.pose.orientation.x = x
            current_pose.pose.orientation.y = y
            current_pose.pose.orientation.z = z

            #publish odom message
            odom = Odometry()
            odom.header.stamp = Clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose = current_pose.pose
            self.kf_odom_publisher.publish(odom)

            #publish path values
            path = Path()
            path.header.stamp = Clock().now().to_msg()
            path.header.frame_id = 'odom'
            self.kf_path_arr.append(current_pose)
            path.poses = self.kf_path_arr
            self.kf_path_publisher.publish(path)






def main(args=None):
    rclpy.init(args=args)
    localizer_node = Localizer()
    rclpy.spin(localizer_node)
    localizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


