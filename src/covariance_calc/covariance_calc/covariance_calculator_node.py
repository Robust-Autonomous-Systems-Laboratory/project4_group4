import rclpy
import numpy as np
import math as m
import statistics as s
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from rclpy.clock import Clock

class Calc(Node): 
    def __init__(self):
        super().__init__('calculator_node')
        subscriber_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_subscription = self.create_subscription(Imu,'/imu', self.imu_callback, subscriber_qos_profile)

        self.ax = []
        self.ay = []
        self.w = []

    def imu_callback(self, msg):
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.w.append(msg.angular_velocity.z)
        if len(self.ax) > 1:
            x = s.variance(self.ax)
            y = s.variance(self.ay)
            w = s.variance(self.w)
            xy = s.covariance(self.ax,self.ay)
            xw = s.covariance(self.ax,self.w)
            yw = s.covariance(self.ay,self.w)
            self.P = np.array([[0,0,0,0,0,0,0,0,0],
                            [0,0,0,0,0,0,0,0,0],
                            [0,0,x,0,0,xy,0,xw,0],
                            [0,0,0,0,0,0,0,0,0],
                            [0,0,0,0,0,0,0,0,0],
                            [0,0,xy,0,0,y,0,yw,0],
                            [0,0,0,0,0,0,0,0,0],
                            [0,0,xw,0,0,yw,0,w,0],
                            [0,0,0,0,0,0,0,0,0]])
            print("x^2 = ", x)
            print("y^2 = ", y)
            print("w^2 = ", w)
            print("xy = ", xy)
            print("xw = ", xw)
            print("yw = ", yw)
            print("\n")

def main(args=None):
    rclpy.init(args=args)
    calculator_node = Calc()
    rclpy.spin(calculator_node)
    calculator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

