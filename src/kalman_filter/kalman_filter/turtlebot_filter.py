import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path, Odometry

class Localizer(Node):
    def __init__(self):
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        super().__init__('localizer_node')

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
        self.JOINT_STATE_IS_FRESH = True
    
    def imu_callback(self, msg):
        self.imu_ang_x_vel = msg.angular_velocity.x
        self.imu_ang_y_vel = msg.angular_velocity.y
        self.imu_ang_z_vel = msg.angular_velocity.z

        self.imu_x_acc = msg.linear_acceleration.x
        self.imu_y_acc = msg.linear_acceleration.y
        self.imu_z_acc = msg.linear_acceleration.z
        self.IMU_IS_FRESH = True

    def imu_callback(self, msg):
        self.cmd_vel_x = msg.twist.linear.x
        self.cmd_vel_x = msg.twist.linear.y
        self.cmd_vel_x = msg.twist.linear.z
        self.cmd_vel_t = msg.twist.angular.z
        self.CMD_VEL_IS_FRESH = True


