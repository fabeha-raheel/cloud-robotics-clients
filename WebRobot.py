#!/usr/bin/python

import rospy
import tf
import websocket # pip install websocket-client
import time
import cv2
import threading
import base64
import json

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped

from WebRobot_Data import *

class WebRobot_ROS():

    def __init__(self, name='no-name', type=None, owner_id=-1, access_key=None) -> None:
        
        self.name = name
        self.type = type
        self.owner_id = owner_id
        self.access_key = access_key
        
        self.data = WebRobot_Data()
        self.kill = False
        self.camera = 0

        self.ws_connected = False
        self.ws_timer = 0
        self.ws_uri = None

    def connect(self, ws_uri=None):
        if ws_uri is not None:
            self.ws_thread = threading.Thread(target=self._ws_thread_target, daemon=True)
            self.ws_thread.start()

    def set_subscribers(self, odom_topic=None, imu_topic=None, global_position_topic=None, camera_topic=None):
        # odom_subscriber should publish data of the type nav_msgs/Odometry
        # imu_subscriber should publish data of the type sensor_msgs/Imu
        # global_position_subscriber should publish data of the type sensor_msgs/NavSatFix

        self.odom_topic = odom_topic
        self.imu_topic = imu_topic
        self.global_position_topic = global_position_topic
        self.camera_topic = camera_topic

    def set_publishers(self, control_type=None, control_topic=None):
        # control_type can be either 'cmd_vel' or 'rc'

        self.control_type = control_type
        self.control_topic = control_topic

    def init_robot(self, video=False):
        
        rospy.init_node(self.data.header.robot_name + '_node', anonymous=True)
        
        self._init_subscribers()
        self._init_publishers()
        if video:
            success = self._init_video()
        else:
            success = True
        
        if success:
            print(self.data.header.robot_name + " of type " + self.data.header.robot_type + " successfully initialized. ")
        else:
            print("Cannot start video.")

    def _init_subscribers(self):
        if self.odom_topic is not None:
            rospy.Subscriber(self.odom_topic, Odometry, self.odomcb)
        else:
            print("No odom topic specified.")
        
        if self.imu_topic is not None:
            rospy.Subscriber(self.imu_topic, Imu, self.imucb)
        else:
            print("No imu topic specified.")

        if self.global_position_topic is not None:
            rospy.Subscriber(self.global_position_topic, NavSatFix, self.global_position_cb)
        else:
            print("No global position topic specified.")

    def _init_publishers(self):
        
        if self.control_type == 'cmd_vel' and self.control_topic is not None:
            self.cmdvel_pub = rospy.Publisher(self.control_topic, Twist, queue_size=10)
        else:
            print("No control topic specified.")

    def _ws_thread_target(self):
        if not self.ws_connected:
            self.websocket = websocket.WebSocketApp(self.ws_uri,
                                on_message = self.ws_on_message,
                                on_error = self.ws_on_error,
                                on_close = self.ws_on_close)
            
            self.websocket.on_open = self.ws_on_open
            
            self.websocket.run_forever(ping_interval=13, ping_timeout=10)

    def reconnect(self):
        print("Attempting to reconnect.............")
        self.ws_thread = threading.Thread(target=self._ws_thread_target, daemon=True)
        self.ws_thread.start()

    def ws_on_close(self):   
        self.ws_connected = False
        print("Websocket Connection Closed.");
        self.ws_timer += 1
        print("Try to reconnect after", self.ws_timer)
        time.sleep(self.ws_timer)
        self.reconnect()

    def ws_on_open(self):
        self.ws_timer = 0
        print("Websocket connection established.")
        time.sleep(1) # to make sure no thread is sending now
        self.ws_connected = True

        self.data_transmission_thread = threading.Thread(target=self._data_transmission_target, daemon=True)
        self.data_transmission_thread.start()

    def ws_on_message(self, message):
        self.message_handler(message)

    def message_handler(self, message):
        print(message)

    def ws_on_error(self, error):   
        print("Websocket Error: ", error)

    def _data_transmission_target(self):
        while self.ws_connected:
            self.websocket.send(json.dumps(self.robot_data))
            time.sleep(0.1)

        print("Data transmission thread stopped")

    def odomcb(self, mssg):

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
        self.data.global_position.altitude = round(mssg.altitude, 2)

    def _init_video(self):
        # Open the webcam and capture video frames
        self.cap = cv2.VideoCapture(self.camera)

        if not self.cap.isOpened():
            print("Error: Unable to access the webcam.")
            return False
        
        # Get the width and height of the video frame
        self.data.video.width = int(self.cap.get(3))
        self.data.video.height = int(self.cap.get(4))

        self._start_video_thread()
        
        return True

    def _start_video_thread(self):
        self.video_thread = threading.Thread(target=self._video_thread_target, daemon=True)
        self.video_thread.start()

    def _video_thread_target(self):
        while not self.kill:
            # Read a frame from the webcam
            ret, frame = self.cap.read()

            # Convert the frame to JPEG format
            _, buffer = cv2.imencode('.jpg', frame)
            self.data.video.frame = base64.b64encode(buffer).decode('utf-8')

    def robot_data(self):
        return self.data.to_dict()
    
    def get_available_robot_types(self):
        return RobotTypes
    
    def end_threads(self):
        self.kill = True
    

if __name__ == '__main__':

    tb3 = WebRobot_ROS()
    print(tb3.data.to_dict())
