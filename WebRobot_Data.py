RobotTypes = ['ArduCopter', 'ArduRover', 'Turtlebot']

class Header:
    def __init__(self) -> None:
        self.robot_name = 'no-name'
        self.robot_type = None

class LocalPosition:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class GlobalPosition:
    def __init__(self, latitude=0, longitude=0, altitude=0):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

class EulerOrientation:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class LinearAcceleration:
    def __init__(self, ax=0, ay=0, az=0):
        self.ax = ax
        self.ay = ay
        self.az = az

class AngularVelocity:
    def __init__(self, wx=0, wy=0, wz=0):
        self.wx = wx
        self.wy = wy
        self.wz = wz

class LinearVelocity:
    def __init__(self, vx=0, vy=0, vz=0):
        self.vx = vx
        self.vy = vy
        self.vz = vz

# class Odom_Data:
#     def __init__(self) -> None:
#         self.local_position = LocalPosition()
#         self.orientation_euler = EulerOrientation()
#         self.linear_velocity = LinearVelocity()
#         self.angular_velocity = AngularVelocity()

# class IMU_Data:
#     def __init__(self):
#         self.orientation_euler = EulerOrientation()
#         self.angular_velocity = AngularVelocity()
#         self.linear_acceleration = LinearAcceleration()

class WebRobot_Data:
    def __init__(self):
        self.header = Header()
        self.local_position = LocalPosition()
        self.global_position = GlobalPosition()
        self.euler_orientation = EulerOrientation()
        self.linear_velocity = LinearVelocity()
        self.angular_velocity = AngularVelocity()
        self.linear_acceleration = LinearAcceleration()


    def to_dict(self):
        return {
            key: value.__dict__
            for key, value in self.__dict__.items()
        }

# Example usage
data = WebRobot_Data()
data_dict = data.to_dict()
print(data_dict)