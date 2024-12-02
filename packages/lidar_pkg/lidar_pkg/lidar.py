import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class  Lidar(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('lidar')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class  Lidar(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('lidar')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0

        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359]
        self.laser_frontLeft = min(msg.ranges[0:15])
        self.laser_frontRight = min(msg.ranges[345:359])


        
    def motion(self):
        # print the data
        self.get_logger().info('Forward: "%s"' % str(self.laser_forward))
        
        if (self.laser_frontLeft < 0.5) :
            self.get_logger().info('Object front left: "%s"' % str(self.laser_frontLeft))
        if (self.laser_frontRight < 0.5) :
            self.get_logger().info('Object front right: "%s"' % str(self.laser_frontRight))

        # Logic of move
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        # Publishing the cmd_vel values to a Topic
        # self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    lidar = Lidar()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(lidar)
    # Explicity destroy the node
    lidar.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359] 
        
        
    def motion(self):
        # print the data
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # Logic of move
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.1
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    lidar = Lidar()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(lidar)
    # Explicity destroy the node
    lidar.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()