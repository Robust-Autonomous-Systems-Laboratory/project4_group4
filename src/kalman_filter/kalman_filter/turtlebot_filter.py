import rclpy
import numpy as np
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from kalman_filter.kalman import KalmanFilter, ExtendedKalmanFilter, UnscentedKalmanFilter
from kalman_filter.kalman_prams import Kf_prams,Ekf_prams, Ukf_prams
from rclpy.clock import Clock
import csv
from kalman_filter.euler_quaternion import convertEulerToQuaternion

PUBLISHER_FREQUENCY_HZ = 20
ROBOT_WHEELBASE = 0.16
ROBOT_WHEEL_RADIUS = 0.033
OUTPUT_PATH = "data/"
class Localizer(Node): 
    def __init__(self):
        super().__init__('localizer_node')
        
        self.delta_t = 1/PUBLISHER_FREQUENCY_HZ
        self.timer = self.create_timer(self.delta_t, self.timer_callback)

        self.kf = Kf_prams(self.delta_t)
        self.ekf = Ekf_prams(self.delta_t)
        self.ukf = Ukf_prams(self.delta_t)
        self.kalman_filter = KalmanFilter(self.kf.F, self.kf.H, self.kf.Q, self.kf.R, self.kf.B, self.kf.x0, self.kf.P0)
        self.extended_kalman_filter = ExtendedKalmanFilter(self.ekf.F, self.ekf.H, self.ekf.Q, self.ekf.R, self.ekf.B, self.ekf.x0, self.ekf.P0)
        self.unscented_kalman_filter = UnscentedKalmanFilter(self.ukf.F, self.ukf.H, self.ukf.Q, self.ukf.R, self.ukf.x0, self.ukf.P0,self.delta_t)

        subscriber_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_subscription = self.create_subscription(JointState,'/joint_states', self.joint_state_callback, subscriber_qos_profile)
        self.FRESH_JOINT_STATE = False
        self.imu_subscription = self.create_subscription(Imu,'/imu', self.imu_callback, subscriber_qos_profile)
        self.FRESH_IMU = False
        self.cmd_vel_subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmd_vel_callback, subscriber_qos_profile)
        self.FRESH_CMD_VEL = False    

        publisher_qos_profile = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.kf_path_publisher = self.create_publisher(Path, 'localization_node/kf/path', publisher_qos_profile)
        self.kf_odom_publisher = self.create_publisher(Odometry, 'localization_node/kf/odometry', publisher_qos_profile)
        self.kf_path_arr = []

        self.ekf_path_publisher = self.create_publisher(Path, 'localization_node/ekf/path', publisher_qos_profile)
        self.ekf_odom_publisher = self.create_publisher(Odometry, 'localization_node/ekf/odometry', publisher_qos_profile)
        self.ekf_path_arr = []

        self.ukf_path_publisher = self.create_publisher(Path, 'localization_node/ukf/path', publisher_qos_profile)
        self.ukf_odom_publisher = self.create_publisher(Odometry, 'localization_node/ukf/odometry', publisher_qos_profile)
        self.ukf_path_arr = []

        self.js_right_wheel = 0
        self.js_left_wheel = 0
        self.imu_w = 0
        self.imu_ax = 0
        self.imu_ay = 0
        self.cmd_vel_x = 0
        self.cmd_vel_y = 0
        self.cmd_vel_w = 0

        kf_output_file = open(OUTPUT_PATH+'kf_output_file.csv', 'w', newline='')
        ekf_output_file = open(OUTPUT_PATH+'ekf_output_file.csv', 'w', newline='')
        ukf_output_file = open(OUTPUT_PATH+'ukf_output_file.csv', 'w', newline='')
        self.kf_writer = csv.writer(kf_output_file)
        self.ekf_writer = csv.writer(ekf_output_file)
        self.ukf_writer = csv.writer(ukf_output_file)
        self.output_t = 0
        header =     ['t','Xx1','Xx2','Xx3','Xy1','Xy2','Xy3','Xt1','Xt2','Xt3',
                          'Zx1','Zx2','Zx3','Zy1','Zy2','Zy3','Zt1','Zt2','Zt3',
                          'Rx1','Rx2','Rx3','Ry1','Ry2','Ry3','Rt1','Rt2','Rt3',
                          'P11','P12','P13','P14','P15','P16','P17','P18','P19',
                          'P21','P22','P23','P24','P25','P26','P27','P28','P29',
                          'P31','P32','P33','P34','P35','P36','P37','P38','P39',
                          'P41','P42','P43','P44','P45','P46','P47','P48','P49',
                          'P51','P52','P53','P54','P55','P56','P57','P58','P59',
                          'P61','P62','P63','P64','P65','P66','P67','P68','P69',
                          'P71','P72','P73','P74','P75','P76','P77','P78','P79',
                          'P81','P82','P83','P84','P85','P86','P87','P88','P89',
                          'P91','P92','P93','P94','P95','P96','P97','P98','P99',]
        self.kf_writer.writerow(header)
        self.ekf_writer.writerow(header)
        self.ukf_writer.writerow(header)

    def joint_state_callback(self, msg):
        self.js_right_wheel = msg.position[1]
        self.js_left_wheel = msg.position[0]
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
        self.kf_u = np.array([[0],
                      [world_x_vel],
                      [0],
                      [0],
                      [world_y_vel],
                      [0],
                      [0],
                      [world_w],
                      [0]])
        return self.kalman_filter.predict(self.kf_u)

    def kf_update(self):
        theta = self.kalman_filter.x[6,0]
        #H = self.kf.update_prams(theta)
        world_t = ((ROBOT_WHEEL_RADIUS*(self.js_right_wheel-self.js_left_wheel))/(ROBOT_WHEELBASE))-(m.pi/2)
        world_w = self.cmd_vel_w
        x_acc = (self.imu_ax*m.cos(theta)) - (self.imu_ax*m.sin(theta))
        y_acc = (self.imu_ax*m.sin(theta)) + (self.imu_ax*m.cos(theta))
        self.kf_z = np.array([[0],
                      [0],
                      [x_acc],
                      [0],
                      [0],
                      [y_acc], 
                      [world_t],
                      [world_w],
                      [0]])
        return self.kalman_filter.update(self.kf_z)

    def ekf_predict(self):
        theta = self.extended_kalman_filter.x[6,0]
        world_x_vel = (self.cmd_vel_x*m.cos(theta)) - (self.cmd_vel_y*m.sin(theta))
        world_y_vel = (self.cmd_vel_x*m.sin(theta)) + (self.cmd_vel_y*m.cos(theta))
        world_w = self.cmd_vel_w
        self.ekf_u = np.array([[0],
                      [world_x_vel],
                      [0],
                      [0],
                      [world_y_vel],
                      [0],
                      [0],
                      [world_w],
                      [0]])
        return self.extended_kalman_filter.predict(self.ekf_u)

    def ekf_update(self):
        theta = self.extended_kalman_filter.x[6,0]
        x_acc = self.extended_kalman_filter.x[2,0]
        y_acc = self.extended_kalman_filter.x[5,0]
        H = self.ekf.update_prams(theta,x_acc,y_acc)
        world_t = ((ROBOT_WHEEL_RADIUS*(self.js_right_wheel-self.js_left_wheel))/(ROBOT_WHEELBASE))-(m.pi/2)
        world_w = self.cmd_vel_w
        self.ekf_z = np.array([[0],
                      [0],
                      [self.imu_ax],
                      [0],
                      [0],
                      [self.imu_ay], 
                      [world_t],
                      [world_w],
                      [0]])
        return self.extended_kalman_filter.update(self.ekf_z,H)
    
    def ukf_predict(self):
        theta = self.unscented_kalman_filter.x[6,0]
        world_x_vel = (self.cmd_vel_x*m.cos(theta)) - (self.cmd_vel_y*m.sin(theta))
        world_y_vel = (self.cmd_vel_x*m.sin(theta)) + (self.cmd_vel_y*m.cos(theta))
        world_w = self.cmd_vel_w
        self.ukf_u = np.array([[0],
                      [world_x_vel],
                      [0],
                      [0],
                      [world_y_vel],
                      [0],
                      [0],
                      [world_w],
                      [0]])
        return self.unscented_kalman_filter.predict()

    def ukf_update(self):
        theta = self.unscented_kalman_filter.x[6,0]
        x_acc = self.unscented_kalman_filter.x[2,0]
        y_acc = self.unscented_kalman_filter.x[5,0]
        H = self.ukf.update_prams(theta,x_acc,y_acc)
        world_t = ((ROBOT_WHEEL_RADIUS*(self.js_right_wheel-self.js_left_wheel))/(ROBOT_WHEELBASE))-(m.pi/2)
        world_w = self.cmd_vel_w
        self.ukf_z = np.array([[0],
                      [0],
                      [self.imu_ax],
                      [0],
                      [0],
                      [self.imu_ay], 
                      [world_t],
                      [world_w],
                      [0]])
        return self.unscented_kalman_filter.update(self.ukf_z,H)

    def timer_callback(self):
        if(self.FRESH_JOINT_STATE and self.FRESH_IMU):
            kf_x_prediction = self.kf_predict()
            kf_x_update, kf_residual, kf_P = self.kf_update()
            ekf_x_prediction = self.ekf_predict()
            ekf_x_update, ekf_residual, ekf_P = self.ekf_update()
            ukf_x_prediction = self.ukf_predict()
            ukf_x_update, ukf_residual, ukf_P = self.ukf_update()
            self.FRESH_JOINT_STATE = False
            self.FRESH_IMU = False
            self.FRESH_CMD_VEL = False
            self.data_writer(self.kf_writer, kf_x_update, self.kf_z, kf_residual, kf_P)
            self.data_writer(self.ekf_writer, ekf_x_update, self.ekf_z, ekf_residual, ekf_P)
            self.data_writer(self.ukf_writer, ukf_x_update, self.ukf_z, ukf_residual, ukf_P)
            self.output_t += 1
            kf_odom, kf_path, self.kf_path_arr = self.message_generator(kf_x_update[0,0],kf_x_update[3,0],kf_x_update[6,0],self.kf_path_arr)
            ekf_odom, ekf_path, self.ekf_path_arr = self.message_generator(ekf_x_update[0,0],ekf_x_update[3,0],ekf_x_update[6,0],self.ekf_path_arr)
            ukf_odom, ukf_path, self.ukf_path_arr = self.message_generator(ukf_x_update[0,0],ukf_x_update[3,0],ukf_x_update[6,0],self.ukf_path_arr)
            self.kf_path_publisher.publish(kf_path)
            self.kf_odom_publisher.publish(kf_odom)
            self.ekf_path_publisher.publish(ekf_path)
            self.ekf_odom_publisher.publish(ekf_odom)
            self.ukf_path_publisher.publish(ukf_path)
            self.ukf_odom_publisher.publish(ukf_odom)

    def data_writer(self,writer,x,z,r,P):
        row =[self.output_t, x[0,0], x[1,0], x[2,0], x[3,0], x[4,0], x[5,0], x[6,0], x[7,0], x[8,0],
                             z[0,0], z[1,0], z[2,0], z[3,0], z[4,0], z[5,0], z[6,0], z[7,0], z[8,0],
                             r[0,0], r[1,0], r[2,0], r[3,0], r[4,0], r[5,0], r[6,0], r[7,0], r[8,0],
                             P[0,0], P[0,1], P[0,2], P[0,3], P[0,4], P[0,5], P[0,6], P[0,7], P[0,8],
                             P[1,0], P[1,1], P[1,2], P[1,3], P[1,4], P[1,5], P[1,6], P[1,7], P[1,8],
                             P[2,0], P[2,1], P[2,2], P[2,3], P[2,4], P[2,5], P[2,6], P[2,7], P[2,8],
                             P[3,0], P[3,1], P[3,2], P[3,3], P[3,4], P[3,5], P[3,6], P[3,7], P[3,8],
                             P[4,0], P[4,1], P[4,2], P[4,3], P[4,4], P[4,5], P[4,6], P[4,7], P[4,8],
                             P[5,0], P[5,1], P[5,2], P[5,3], P[5,4], P[5,5], P[5,6], P[5,7], P[5,8],
                             P[6,0], P[6,1], P[6,2], P[6,3], P[6,4], P[6,5], P[6,6], P[6,7], P[6,8],
                             P[7,0], P[7,1], P[7,2], P[7,3], P[7,4], P[7,5], P[7,6], P[7,7], P[7,8],
                             P[8,0], P[8,1], P[8,2], P[8,3], P[8,4], P[8,5], P[8,6], P[8,7], P[8,8]]
        writer.writerow(row)
        
 
    def message_generator(self,x,y,t,path_arr):
            #generate pose message
            current_pose = PoseStamped()
            current_pose.header.stamp = Clock().now().to_msg()
            current_pose.header.frame_id = 'odom'
            current_pose.pose.position.x = x
            current_pose.pose.position.y = y
            #print("x = ",current_pose.pose.position.x)
            #print("y = ",current_pose.pose.position.y)
            #print("\n")
            #get orientation quaternion
            w,x,y,z = convertEulerToQuaternion(0,0,t)
            current_pose.pose.orientation.w = w
            current_pose.pose.orientation.x = x
            current_pose.pose.orientation.y = y
            current_pose.pose.orientation.z = z

            #get odom message
            odom = Odometry()
            odom.header.stamp = Clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose = current_pose.pose

            #get path values
            path = Path()
            path.header.stamp = Clock().now().to_msg()
            path.header.frame_id = 'odom'
            path_arr.append(current_pose)
            path.poses = path_arr
            return odom, path, path_arr


def main(args=None):
    rclpy.init(args=args)
    localizer_node = Localizer()
    rclpy.spin(localizer_node)
    localizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


