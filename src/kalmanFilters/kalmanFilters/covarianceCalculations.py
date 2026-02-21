import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
import statistics as stats

class covarCalc(Node):
    def __init__(self):
        super().__init__('covarCalc_node')
        print("test2\n")
        subscriber_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_subscription = self.create_subscription(Imu,'/imu', self.imu_callback, 10)
        #initialize arrays
        self.aX = []
        self.aY = []
        self.w = []
        print("test3\n")


    def imu_callback(self, msg):
        print("test4\n")
        self.aX.append(msg.linear_acceleration.x)
        self.aY.append(msg.linear_acceleration.y)
        self.w.append(msg.angular_velocity.z)
        print("\n", self.w )
        if len(self.aX) > 1:
            xx = stats.variance(self.aX)
            yy = stats.variance(self.aY)
            ww = stats.variance(self.w)
            xy = stats.covariance(self.aX, self.aY)
            xw = stats.covariance(self.aX, self.w)
            yw = stats.covariance(self.aY, self.w)

            #Output covariance results
            print("xx = ", xx)
            print("yy = ", yy)
            print("ww = ", ww)
            print("xy = ", xy)
            print("xw = ", xw)
            print("yw = ", yw)
            print("\n")

def main(args=None):
    print("test1 \n")
    rclpy.init(args = args)
    covarCalc_node = covarCalc()
    try:
        while rclpy.ok():
            print("test 6 \n")
            rclpy.spin(covarCalc_node)
            print("test 7 \n")
    except:
        covarCalc_node.destroy_node()
        rclpy.shutdown()
    print("test5 \n")

if __name__ == '__main__':
    main()