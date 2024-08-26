#!/usr/bin/python

import rospy
import websockets
import json
import asyncio
import time
import signal
import sys

from websockets.exceptions import ConnectionClosedError
from WebRobot import WebRobot

# ROBOT_NAME = 'Turtlebot3'
# ROBOT_TYPE = 'Turtlebot'
# ROBOT_SLUG = 'tb3'

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

            # Define a coroutine to handle sending and receiving data
            async def send_receive_data():
                while not rospy.is_shutdown():
                    try:
                        # Send data
                        data = tb3_robot.robot_data()
                        data_to_send = {
                            'cmdtype': 'tb3data',
                            'data': data,
                        }
                        print("Sending data...")
                        # print("Sending data:", data_to_send)
                        await websocket.send(json.dumps(data_to_send))

                        # Receive data
                        data_received = await websocket.recv()
                        # print("Received data:", data_received)
                        
                        # Process received data here

                        event = json.loads(data_received)

                        if event['cmdtype'] == 'cmd_vel':
                            class cmd_container:
                                x = 0
                                y = 0
                                z = 0
        
                            linear_cmd = cmd_container()
                            angular_cmd = cmd_container()

                            print("Publishing Cmd_vel")
                            linear_cmd.x = event['data']['linear_x']
                            linear_cmd.y = event['data']['linear_y']
                            linear_cmd.z = event['data']['linear_z']
                            angular_cmd.x = event['data']['angular_x']
                            angular_cmd.y = event['data']['angular_y']
                            angular_cmd.z = event['data']['angular_z']
                            tb3_robot.move_robot(linear_cmd, angular_cmd)
                            print("Received data:", data_received)
                            time.sleep(0.2)

                    except ConnectionClosedError as e:
                        print("Caught the connection closed error.")
                        print(e)
                        break
                    except Exception as e:
                        print("Websocket Exception.")
                        print(e)
                        break

            # Start the send_receive_data coroutine
            await send_receive_data()

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