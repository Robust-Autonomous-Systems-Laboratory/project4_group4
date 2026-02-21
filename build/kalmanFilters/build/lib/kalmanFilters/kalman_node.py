import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import math as m
import statistics as stats

#Robot Parameters
PUBLISHER_FREQ = 20 #Hz
WHEELBASE = 0.16 #m
WHEEL_RADIUS = 0.033 #m

class kalmanFilters(Node):
    def __init__(self):
        super().__init__('kf_node')

        self.delta_t = 1/PUBLISHER_FREQ
        self.timer = self.create_timer(self.delta_t, self.timerCallback)

        
        subscriber_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.jointStateSub = self.create_subscription(JointState, '/joint_states', self.jointStateCallback, subscriber_qos_profile)
        self.FRESH_JOINT_STATE = False
        self.imuSub = self.create_subscription(Imu, '/imu', self.imuCallback, subscriber_qos_profile)
        self.FRESH_IMU = False
        self.cmdVelSub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmdVelCallback, subscriber_qos_profile)
        self.FRESH_VEL = False

        publisher_qos_profile = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.kfPathPub = self.create_publisher(Path, 'localization_node/kf/path', publisher_qos_profile)
        self.kfOdomPub = self.create_publisher(Odometry, 'localization_node/kf/odometry', publisher_qos_profile)
        self.kfPathArray = []

        #Initializations
        self.jsRightWheel = 0
        self.jsLeftWheel = 0
        self.imuW = 0
        self.imuAx = 0
        self.imuAy = 0
        self.cmdVelX = 0
        self.cmdVelY = 0
        self.cmdVelW = 0

        self.KF = KFParams(self.delta_t, self.imuAx, self.imuAy, self.imuW)
        self.KalmanFilter = KalmanFilter(self.KF.F, self.KF.B, self.KF.H, self.KF.Q, self.KF.R , self.KF.x0, self.KF.P0)

    def jointStateCallback(self, msg):
        self.jsRightWheel = msg.position[1]
        self.jsLeftWheel = msg.position[0]
        self.FRESH_JOINT_STATE = True
        
    def imuCallback(self, msg):
        self.imuW = msg.angugular_velocity.z
        self.imuAx = msg.linear_acceleration.x
        self.imuAy = msg.linear_acceleration.y
        self.FRESH_IMU = True

    def cmdVelCallback(self, msg):
        self.cmdVelX = msg.twist.linear.x
        self.cmdVelY = msg.twist.linear.y
        self.cmdVelW = msg.twist.angular.z
        self.FRESH_VEL = True
        
        #Normal Kalman Filter Functions
    def kfPredict(self):
        #Rotation of robot frame relative to world frame
        theta = self.KalmanFilter.x[6,0]
        #Robot motion relative to world frame
        worldXVel = (self.cmdVelX*m.cos(theta)) - (self.cmdVelY*m.sin(theta))
        worldYVel = (self.cmdVelX*m.sin(theta)) + (self.cmdVelY*m.cos(theta))
        worldW = self.cmdVelW
        #control vector
        u = np.array([[0], [worldXVel], [0], [0], [worldYVel], [0], [0], [worldW], [0]])
        return self.KalmanFilter.predict(u)
        
    def kfUpdate(self):
        #Rotation of robot frame relative to world frame
        theta = self.KalmanFilter.x[6,0]
        #Robot motion relative to world
        worldTheta = ((WHEEL_RADIUS*(self.jsRightWheel-self.jsLeftWheel))/(WHEELBASE))-(m.pi/2)
        worldW = self.cmdVelW
        worldXAccel = (self.imuAx*m.cos(theta)) - (self.imuAy*m.sin(theta))
        worldYAccel = (self.imuAx*m.sin(theta)) + (self.imuAy*m.cos(theta))
        #measurement vector
        z = np.array([[0],[0],[worldXAccel], [0], [0], [worldYAccel], [worldTheta], [worldW], [0]])
        return self.KalmanFilter.update(z)
        
    #Extended Kalman Filter Functions

    #Unscented Kalman Filter Functions

    def timerCallback(self):
        if(self.FRESH_JOINT_STATE and self.FRESH_IMU):
            kfXPrediction = self.kfPredict()
            kfXUpdate = self.kfUpdate()
            self.FRESH_JOINT_STATE = False
            self.FRESH_IMU = False
            self.FRESH_VEL = False

            kfOdom, kfPath, self.kfPathArray = self.messageGenerator(kfXUpdate[0,0], kfXUpdate[3,0], kfXUpdate[6,0],self.kfPathArray)
        
            self.kfPathPub(kfPath)
            self.kfOdomPub(kfOdom)

    def messageGenerator(self, x, y, t, pathArray):
        currentPose = PoseStamped()
        currentPose.header.stamp = Clock().now().to_msg()
        currentPose.header.frame_id = 'odom'
        currentPose.pose.position.x = x
        currentPose.pose.position.y = y

        #convert orientation to quaternion
        w, x, y, z = euler2Quaternion(0,0,t)
        currentPose.pose.orientation.w = w
        currentPose.pose.orientation.x = x
        currentPose.pose.orientation.y = y
        currentPose.pose.orientation.z = z

        odom = Odometry()
        odom.header.stamp = Clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose = currentPose.pose

        path = Path()
        path.header.stamp = Clock().now().to_msg()
        path.header.frame_id = 'odom'
        pathArray.append(currentPose)
        path.poses = pathArray
            
        return odom, path, pathArray



#Normal Kalman Filter Parameters
class KFParams:
    def __init__(self, dt, aX, aY, w):
        self.F = np.array([[1, dt, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0.5, dt, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, dt, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0.5, dt, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, dt, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0.5, dt],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1],])
        self.B = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0.5, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0.5, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0.5, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],])
        self.H = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],])
        self.Q = np.identity(9)
        #Sensor noise covariance values (IMU):
        #xx = stats.variance(aX)
        #yy = stats.variance(aY)
        #ww = stats.variance(w)
        #xy = stats.covariance(aX, aY)
        #xw = stats.covariance(aX, w)
        #yw = stats.covariance(aY, w)
        self.R = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1],])
        self.x0 = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],])
        self.P0 = np.identity(9)
    
#Extended Kalman Filter Parameters

#Unscented Kalman Filter Parameters

#Normal Kalman Filter
class KalmanFilter:
    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F = F  # State transition matrix (system model).
        self.B = B  # Control matrix (effect of control input).
        self.H = H  # Observation matrix (how we measure the state).
        self.Q = Q  # Process noise covariance (uncertainty in the process).
        self.R = R  # Measurement noise covariance (uncertainty in the measurements).
        self.x = x0 # Initial state estimate.
        self.P = P0 # Initial error covariance (initial uncertainty of state estimate).

    def predict(self, u):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(self.F, np.dot(self.P, np.transpose(self.F))) + self.Q

    def update(self,z):
        S = np.dot(self.H, np.dot(self.P, np.transpose(self.H))) + self.R
        K = np.dot(np.dot(self.P, np.transpose(self.H)), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        residual = z - np.dot(self.H, self.x)
        return self.x, residual

#Extended Kalman Filter

#Unscented Kalman Filter
    

#Helper function to convert euler angles to quaternions
def euler2Quaternion(roll, pitch, yaw):
    w = m.cos(roll/2)*m.cos(pitch/2)*m.cos(yaw/2) + m.sin(roll/2)*m.sin(pitch/2)*m.sin(yaw/2)
    x = m.sin(roll/2)*m.cos(pitch/2)*m.cos(yaw/2) - m.cos(roll/2)*m.sin(pitch/2)*m.sin(yaw/2)
    y = m.cos(roll/2)*m.sin(pitch/2)*m.cos(yaw/2) + m.sin(roll/2)*m.cos(pitch/2)*m.sin(yaw/2)
    z = m.cos(roll/2)*m.cos(pitch/2)*m.sin(yaw/2) - m.sin(roll/2)*m.sin(pitch/2)*m.cos(yaw/2)
    return w, x, y, z

def main(args=None):
    rclpy.init(args=args)
    kf_node = kalmanFilters()
    try:
        while rclpy.ok():
            rclpy.spin(kf_node)
    except KeyboardInterrupt:
        kf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
