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

ROBOT_NAME = 'Iris'
ROBOT_TYPE = 'ArduCopter'
ROBOT_SLUG = 'iris'

WS_URI = "ws://localhost:8000/ws/" + ROBOT_SLUG + "/"

KILL_EVENT = asyncio.Event()
OK_EVENT = asyncio.Event()

async def main():
    robot = WebRobot()
    robot.set_robot_name(ROBOT_NAME)
    try:
        robot.set_robot_type(ROBOT_TYPE)
    except:
        print("Please select a valid robot type. Choose from: ", robot.get_available_robot_types())
        sys.exit(1)
    robot.init_robot(video=True)

    while not rospy.is_shutdown():
        try:
            async with websockets.connect(WS_URI) as websocket:
                print("WebSocket connection established")

                print('Waiting for data...')
                time.sleep(0.5)

                while not rospy.is_shutdown():
                    data_to_send = robot.robot_data()
                    print("Sending data...")
                    # print(data_to_send)

                    try:
                        await websocket.send(json.dumps(data_to_send))
                    except ConnectionClosedError as e:
                        print("Caught the connection closed error.")
                        break  # Exit the inner loop if connection is closed
                    except Exception as e:
                        print("Websocket Exception.")
                        break
                
                    await asyncio.sleep(0.1)
                    if KILL_EVENT.is_set():
                        print("Kill raised - Breaking while loop...")
                        break  # Exit the inner loop if kill event is set
        except ConnectionError as e:
            print(f"Connection error: {e}")
            print("Reconnecting...")
            await asyncio.sleep(2)  # Wait for a few seconds before attempting to reconnect

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
