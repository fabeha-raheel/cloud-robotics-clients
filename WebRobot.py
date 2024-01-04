#!/usr/bin/python

import rospy
import tf
import websocket # pip install websocket-client
import time
import cv2
import threading
import base64

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped

from WebRobot_Data import *

class WebRobot():

    def __init__(self) -> None:
        
        # self.ws_uri = ws_uri
        self.data = WebRobot_Data()

    def set_robot_name(self, robot_name):
        self.data.header.robot_name = robot_name

    def set_robot_type(self, robot_type):
        
        if robot_type in RobotTypes:
            self.data.header.robot_type = robot_type

        else:
            raise Exception("Not a valid robot type. Choose from the following supported types: ", RobotTypes)

    def init_robot(self):
        
        rospy.init_node(self.data.header.robot_name + '_node', anonymous=True)
        
        self._init_subscribers()
        self._init_publishers()

        print(self.data.header.robot_name + " of type " + self.data.header.robot_type + " successfully initialized. ")

    def _init_subscribers(self):

        if self.data.header.robot_type == 'Turtlebot':
            rospy.Subscriber('/odom', Odometry, self.odomcb)
            rospy.Subscriber('/imu', Imu, self.imucb)
            
        elif self.data.header.robot_type == 'ArduCopter' or self.data.header.robot_type == 'ArduRover':
            rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_cb)
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_cb)
            rospy.Subscriber('/mavros/imu/data', Imu, self.imucb)
            rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.local_velocity_cb)

    def _init_publishers(self):

        if self.data.header.robot_type == 'Turtlebot':
            self.cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def _init_video(self):
        # Open the webcam and capture video frames
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error: Unable to access the webcam.")
            return
        
        # Get the width and height of the video frame
        self.data.video.width = int(self.cap.get(3))
        self.data.video.height = int(self.cap.get(4))

    def _start_video_thread(self):
        self.video_thread = threading.Thread(target=self._video_thread_target, daemon=True)
        self.video_thread.start()

    def _video_thread_target(self):
        while True:
            # Read a frame from the webcam
            ret, frame = self.cap.read()

            # Convert the frame to JPEG format
            _, buffer = cv2.imencode('.jpg', frame)
            self.data.video.frame = base64.b64encode(buffer).decode('utf-8')

    def init_socket_connection(self):
        pass

    def robot_data(self):
        return self.data.to_dict()
    
    def get_available_robot_types(self):
        return RobotTypes
    
    # Subscriber Callbacks

    def odomcb(self, mssg):
        # self.data.local_position.x = mssg.pose.pose.position.x
        # self.data.local_position.y = mssg.pose.pose.position.y
        # self.data.local_position.z = mssg.pose.pose.position.z

        self.data.local_position.x = round(mssg.pose.pose.position.x, 3)
        self.data.local_position.y = round(mssg.pose.pose.position.y, 3)
        self.data.local_position.z = round(mssg.pose.pose.position.z, 3)

        quaternion = (mssg.pose.pose.orientation.x, mssg.pose.pose.orientation.y, mssg.pose.pose.orientation.z, mssg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.data.euler_orientation.roll = round(euler[0], 3)
        self.data.euler_orientation.pitch = round(euler[1], 3)
        self.data.euler_orientation.yaw = round(euler[2], 3)
        
        self.data.linear_velocity.vx = round(mssg.twist.twist.linear.x, 3)
        self.data.linear_velocity.vy = round(mssg.twist.twist.linear.y, 3)
        self.data.linear_velocity.vz = round(mssg.twist.twist.linear.z, 3)

        self.data.angular_velocity.wx = round(mssg.twist.twist.angular.x, 3)
        self.data.angular_velocity.wy = round(mssg.twist.twist.angular.y, 3)
        self.data.angular_velocity.wz = round(mssg.twist.twist.angular.z, 3)
        
    def imucb(self, mssg):
        self.data.linear_acceleration.ax = round(mssg.linear_acceleration.x, 3)
        self.data.linear_acceleration.ay = round(mssg.linear_acceleration.y, 3)
        self.data.linear_acceleration.az = round(mssg.linear_acceleration.z, 3)

    def global_position_cb(self, mssg):
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        self.data.global_position.altitude = mssg.altitude

    def local_position_cb(self, mssg):
        self.data.local_position.x = round(mssg.pose.position.x, 3)
        self.data.local_position.y = round(mssg.pose.position.y, 3)
        self.data.local_position.z = round(mssg.pose.position.z, 3)

        quaternion = (mssg.pose.orientation.x, mssg.pose.orientation.y, mssg.pose.orientation.z, mssg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.data.euler_orientation.roll = round(euler[0], 3)
        self.data.euler_orientation.pitch = round(euler[1], 3)
        self.data.euler_orientation.yaw = round(euler[2], 3)

    def local_velocity_cb(self, mssg):
        self.data.linear_velocity.vx = round(mssg.twist.linear.x, 3)
        self.data.linear_velocity.vy = round(mssg.twist.linear.y, 3)
        self.data.linear_velocity.vz = round(mssg.twist.linear.z, 3)

        self.data.angular_velocity.wx = round(mssg.twist.angular.x, 3)
        self.data.angular_velocity.wy = round(mssg.twist.angular.y, 3)
        self.data.angular_velocity.wz = round(mssg.twist.angular.z, 3)


if __name__ == '__main__':

    tb3 = WebRobot()
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