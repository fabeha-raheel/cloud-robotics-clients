#!/usr/bin/python

import rospy
import websockets
import json
import asyncio
import time
import signal
from websockets.exceptions import ConnectionClosedError
import sys

from WebRobot import WebRobot
# from WebRobot_Data import *

ROBOT_NAME = 'Turtle1'
ROBOT_TYPE = 'Turtlebot'
ROBOT_SLUG = 'turtle'

# WS_URI = "ws://localhost:8000/ws/" + ROBOT_SLUG + "/"
WS_URI = "ws://192.168.0.112:8000/ws/" + ROBOT_SLUG + "/"

KILL_EVENT = asyncio.Event()
OK_EVENT = asyncio.Event()

async def main():
    tb3_robot = WebRobot()
    tb3_robot.set_robot_name(ROBOT_NAME)
    try:
        tb3_robot.set_robot_type(ROBOT_TYPE)
    except:
        print("Please select a valid robot type. Choose from: ", tb3_robot.get_available_robot_types())
        sys.exit(1)
    tb3_robot.init_robot()

    try:
        async with websockets.connect(WS_URI) as websocket:
            print("WebSocket connection established")

            print('Waiting for data...')
            time.sleep(0.5)

            while not rospy.is_shutdown():
                data_to_send = tb3_robot.robot_data()
                print("Sending data...")
                # print(data_to_send)

                try:
                    await websocket.send(json.dumps(data_to_send))
                except ConnectionClosedError as e:
                    print("Caught the connection closed error.")
                    break
                    # print(e)
                except Exception as e:
                    print("Websocket Exception.")
                    break
                    # print("Error: ", e)
                
                await asyncio.sleep(0.1)
                if KILL_EVENT.is_set():
                    break
        await websocket.close()
        OK_EVENT.set()

    except KeyboardInterrupt:
        print("Ctrl+C pressed. Exiting...")
        await websocket.close()
    except RuntimeError as e:
        # Ignore the "Event loop stopped before Future completed" error
        await websocket.close()
        pass

async def stop_handler():
    global KILL_EVENT, OK_EVENT
    await OK_EVENT.wait()
    KILL_EVENT.set()
    loop = asyncio.get_running_loop()
    loop.stop()

if __name__ == '__main__':
    loop = asyncio.get_event_loop()

    # Set up a signal handler to stop the event loop on Ctrl+C
    loop.add_signal_handler(signal.SIGINT, lambda: asyncio.ensure_future(stop_handler()))

    try:
        # Run the WebSocket client and stop handler concurrently
        main_task = asyncio.ensure_future(main())
        stop_task = asyncio.ensure_future(stop_handler())
        loop.run_until_complete(main_task)
        loop.run_until_complete(stop_task)
    finally:
        loop.close()
