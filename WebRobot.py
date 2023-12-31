#!/usr/bin/python

import rospy
import tf
import websocket # pip install websocket-client
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped

from WebRobot_Data import *

class WebRobot_client():

    def __init__(self) -> None:
        
        # self.ws_uri = ws_uri
        self.data = WebRobot_Data()

    def init_robot(self, robot_name, robot_type):
        
        rospy.init_node(robot_name + '_node', anonymous=True)
        self.data.header.robot_name = robot_name
        
        if robot_type in RobotTypes:
            self.data.header.robot_type = robot_type
        else:
            raise Exception("Not a valid robot type. Choose from the following supported types: ", RobotTypes)
        
        self.init_subscribers()
        self.init_publishers()
        time.sleep(2)
        print(self.data.header.robot_name + " of type " + self.data.header.robot_type + " successfully initialized. ")

    def init_subscribers(self):

        if self.data.header.robot_type == 'Turtlebot':
            rospy.Subscriber('/odom', Odometry, self.odomcb)
            rospy.Subscriber('/imu', Imu, self.imucb)
            
        elif self.data.header.robot_type == 'ArduCopter' or self.data.header.robot_type == 'ArduRover':
            rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_cb)
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_cb)
            rospy.Subscriber('/mavros/imu/data', Imu, self.imucb)
            rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.local_velocity_cb)

    def init_publishers(self):

        if self.data.header.robot_type == 'Turtlebot':
            self.cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def init_socket_connection(self):
        pass

    def return_data(self):
        return self.data.to_dict()
    
    def get_available_robot_types(self):
        return RobotTypes
    
    # Subscriber Callbacks

    def odomcb(self, mssg):
        self.data.local_position.x = mssg.pose.pose.position.x
        self.data.local_position.y = mssg.pose.pose.position.y
        self.data.local_position.z = mssg.pose.pose.position.z

        quaternion = (mssg.pose.pose.orientation.x, mssg.pose.pose.orientation.y, mssg.pose.pose.orientation.z, mssg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.data.euler_orientation.roll = euler[0]
        self.data.euler_orientation.pitch = euler[1]
        self.data.euler_orientation.yaw = euler[2]
        
        self.data.linear_velocity.vx = mssg.twist.twist.linear.x
        self.data.linear_velocity.vy = mssg.twist.twist.linear.y
        self.data.linear_velocity.vz = mssg.twist.twist.linear.z

        self.data.angular_velocity.wx = mssg.twist.twist.angular.x
        self.data.angular_velocity.wy = mssg.twist.twist.angular.y
        self.data.angular_velocity.wz = mssg.twist.twist.angular.z
        
    def imucb(self, mssg):
        self.data.linear_acceleration.ax = mssg.linear_acceleration.x
        self.data.linear_acceleration.ay = mssg.linear_acceleration.y
        self.data.linear_acceleration.az = mssg.linear_acceleration.z

    def global_position_cb(self, mssg):
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        self.data.global_position.altitude = mssg.altitude

    def local_position_cb(self, mssg):
        self.data.local_position.x = mssg.pose.position.x
        self.data.local_position.y = mssg.pose.position.y
        self.data.local_position.z = mssg.pose.position.z

        quaternion = (mssg.pose.orientation.x, mssg.pose.orientation.y, mssg.pose.orientation.z, mssg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.data.euler_orientation.roll = euler[0]
        self.data.euler_orientation.pitch = euler[1]
        self.data.euler_orientation.yaw = euler[2]

    def local_velocity_cb(self, mssg):
        self.data.linear_velocity.vx = mssg.twist.linear.x
        self.data.linear_velocity.vy = mssg.twist.linear.y
        self.data.linear_velocity.vz = mssg.twist.linear.z

        self.data.angular_velocity.wx = mssg.twist.angular.x
        self.data.angular_velocity.wy = mssg.twist.angular.y
        self.data.angular_velocity.wz = mssg.twist.angular.z


if __name__ == '__main__':

    tb3 = WebRobot_client()
    tb3.init_robot(robot_name='tb3', robot_type='Turtlebot')
    print(tb3.data.to_dict())

    # while True:
    #     try:
    #         print(tb3.data.to_dict())
    #         time.sleep(1)
    #     except KeyboardInterrupt:
    #         sys.exit(1)


'''
Next Steps:

1. dictionary WebRobot Data -> send on websockets.
2. modify consumers.py file to accept this data - WebRobotConsumer
3. modify js file to use this data


4. Send video data
5. Test with Iris drone


'''