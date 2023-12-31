#!/usr/bin/python

import rospy
import websockets
import json
import asyncio
import time
import cv2
import base64
from websockets.exceptions import ConnectionClosedError

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


def odomcb(data):
    global pose
    global twist
    pose = data.pose.pose
    twist = data.twist.twist

def imucb(data):
    global orientation
    global angVel
    global linAcc

    orientation = data.orientation
    angVel = data.angular_velocity
    linAcc = data.linear_acceleration


async def send_odom_data(websocket):
    global pose
    global twist

    # rospy.init_node('odom_listener', anonymous=True)

    

    while True:
        odom_data = {
            # 'odom_linear_x': round(pose.position.x, 3),
            # 'odom_linear_y': round(pose.position.y, 3), 
            # 'odom_linear_z': round(pose.position.z, 3),
            # 'odom_angular_x': round(pose.orientation.x, 3),
            # 'odom_angular_y': round(pose.orientation.y, 3),
            # 'odom_angular_z': round(pose.orientation.z, 3),
            # 'odom_angular_w': round(pose.orientation.w, 3)

            'odom_linear_y': pose.position.y, 
            'odom_linear_x': pose.position.x,
            'odom_linear_z': pose.position.z,
            'odom_angular_x': pose.orientation.x,
            'odom_angular_y': pose.orientation.y,
            'odom_angular_z': pose.orientation.z,
            'odom_angular_w': pose.orientation.w
            }

        data_to_send = {
             'type': 'odom_data',
             'data': odom_data
        }  
        # Send data to the server
        try:
            await websocket.send(json.dumps(data_to_send))
        except ConnectionClosedError as e:
            print("Caught the connection closed error.")
            print(e)
        except Exception as e:
            print("Error: ", e)
        await asyncio.sleep(1)

async def send_imu_data(websocket):

    global orientation
    global angVel
    global linAcc

    # rospy.init_node('imu_listener', anonymous=True)

    

    while True:
        imu_data = {
            # 'imu_orient_x': round(orientation.x, 3),
            # 'imu_orient_y': round(orientation.y, 3),
            # 'imu_orient_z': round(orientation.z, 3),
            # 'imu_orient_w': round(orientation.w, 3),
            # 'imu_angVel_x': round(angVel.x, 3),
            # 'imu_angVel_y': round(angVel.y, 3),
            # 'imu_angVel_z': round(angVel.z, 3),
            # 'imu_linearAcc_x': round(linAcc.x, 3),
            # 'imu_linearAcc_y': round(linAcc.y, 3),
            # 'imu_linearAcc_z': round(linAcc.z, 3)

            'imu_orient_x': orientation.x,
            'imu_orient_y': orientation.y,
            'imu_orient_z': orientation.z,
            'imu_orient_w': orientation.w,
            'imu_angVel_x': angVel.x,
            'imu_angVel_y': angVel.y,
            'imu_angVel_z': angVel.z,
            'imu_linearAcc_x': linAcc.x,            
            'imu_linearAcc_y': linAcc.y,
            'imu_linearAcc_z': linAcc.z,
            }
        data_to_send = {
             'type': 'imu_data',
             'data': imu_data
        }

        # # Send data to the server
        try:
            await websocket.send(json.dumps(data_to_send))
        except ConnectionClosedError as e:
            print("Caught the connection closed error.")
            print(e)
        except Exception as e:
            print("Error: ", e)
        
        await asyncio.sleep(1)

async def send_video(websocket):

    # Open the webcam and capture video frames
    video_capture = cv2.VideoCapture(0)
      
    # Check if the webcam is opened correctly
    if not video_capture.isOpened():
        print("Error: Unable to access the webcam.")
        return
    
    # Get the width and height of the video frame
    frame_width = int(video_capture.get(3))
    frame_height = int(video_capture.get(4))

    while True:
        # Read a frame from the webcam
        ret, frame = video_capture.read()

        # Resize Image
        dim = (int(frame_width/2), int(frame_height/2))
        resized = cv2.resize(frame, dim)

        # Convert the frame to JPEG format
        _, buffer = cv2.imencode('.jpg', resized)
        frame_data = base64.b64encode(buffer).decode('utf-8')

        video_data = {
            'frame': frame_data,
            'width': frame_width,
            'height': frame_height
        }

        data_to_send = {
             'type': 'video_data',
             'data': video_data
        }

        # # Send data to the server
        try:
            await websocket.send(json.dumps(data_to_send))
        except ConnectionClosedError as e:
            print("Caught the connection closed error.")
            print(e)
        except Exception as e:
            print("Error: ", e)
        
        await asyncio.sleep(0.1)
    
async def receive_commands(websocket):

    # Create a publisher to move TB3
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    class cmd_container:
        x = 0
        y = 0
        z = 0
    
    linear_cmd = cmd_container()
    angular_cmd = cmd_container()

    while True:

        # Receive message from the server
        message = await websocket.recv()

        event = json.loads(message)

        if event['type'] == 'cmd_vel':
            print("Publishing Cmd_vel")
            linear_cmd.x = event['data']['linear_x']
            linear_cmd.y = event['data']['linear_y']
            linear_cmd.z = event['data']['linear_z']
            angular_cmd.x = event['data']['angular_x']
            angular_cmd.y = event['data']['angular_y']
            angular_cmd.z = event['data']['angular_z']
            move_robot(pub, linear_cmd, angular_cmd)



def move_robot(pub, linear, angular):

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.linear.x = linear.x
    move_cmd.linear.y = linear.y
    move_cmd.linear.z = linear.z
    move_cmd.angular.x = angular.x
    move_cmd.angular.y = angular.y
    move_cmd.angular.z = angular.z

    # Publish cmd_vel
    pub.publish(move_cmd)

async def main():

    global pose
    global twist

    global orientation
    global angVel
    global linAcc

    uri = "ws://13.53.117.181:8000/ws/tb3/"  #production
    # uri = "ws://localhost:8000/ws/tb3/" #development

    #Subscribe to the TB3 /odom topic
    rospy.Subscriber('/odom', Odometry, odomcb)

    #Subscribe to the TB3 /imu topic
    rospy.Subscriber('/imu', Imu, imucb)

    
    
    async with websockets.connect(uri) as websocket:
        print("WebSocket connection established")

        print('Waiting for data...')
        time.sleep(1)

        await asyncio.gather(
            asyncio.create_task(send_odom_data(websocket)),
            asyncio.create_task(send_imu_data(websocket)),
            asyncio.create_task(send_video(websocket)),
            asyncio.create_task(receive_commands(websocket))
        )

        

if __name__ == '__main__':

    rospy.init_node('tb3_client', anonymous=True)
   
    # Run the WebSocket client
    asyncio.get_event_loop().run_until_complete(main())
    rospy.spin()