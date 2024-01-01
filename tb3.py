#!/usr/bin/python

import rospy
import websockets
import json
import asyncio
import time
from websockets.exceptions import ConnectionClosedError
import sys

from WebRobot import WebRobot
# from WebRobot_Data import *

ROBOT_NAME = 'Turtlebot3'
ROBOT_TYPE = 'Turtlebot'
ROBOT_SLUG = 'tb3'

WS_URI = "ws://localhost:8000/ws/" + ROBOT_SLUG + "/"

async def main():

    tb3_robot = WebRobot()
    tb3_robot.set_robot_name(ROBOT_NAME)
    try:
        tb3_robot.set_robot_type(ROBOT_TYPE)
    except:
        print("Please select a valid robot type. Choose from: ", tb3_robot.get_available_robot_types())
        sys.exit(1)
    tb3_robot.init_robot()

    async with websockets.connect(WS_URI) as websocket:
        print("WebSocket connection established")

        print('Waiting for data...')
        time.sleep(1)

        while True:
            data_to_send = tb3_robot.robot_data()
            print("Sending data...")

            # # Send data to the server
            try:
                await websocket.send(json.dumps(data_to_send))
            except ConnectionClosedError as e:
                print("Caught the connection closed error.")
                print(e)
            except Exception as e:
                print("Error: ", e)
            
            await asyncio.sleep(0.1)

if __name__ == '__main__':

    # Run the WebSocket client
    asyncio.get_event_loop().run_until_complete(main())
    rospy.spin()

