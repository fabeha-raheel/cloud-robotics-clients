from WebRobot import WebRobot_ROS

ROBOT_NAME = 'Turtlebot'
ROBOT_SLUG = '/turtle'

WS_URI = ''

odom_sub = ''
imu_sub = ''

cmd_vel_pub = ''


tb3 = WebRobot_ROS(name=ROBOT_NAME, type='Turtlebot')
tb3.set_subscribers(odom_topic=odom_sub, imu_topic=imu_sub)
tb3.set_publishers(control_type='cmd_vel', control_topic=cmd_vel_pub)

tb3.init_robot()
tb3.connect(WS_URI)

