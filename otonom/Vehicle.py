import math
import threading
import time

from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect




class ConnectionStrings:
    SITL = 'udp:127.0.0.1:14551'
    USB = '/dev/ttyACM0'
    TELEMETRY = '/dev/ttyUSB0'


class Vehicle:

    def __init__(self, communication=None):
        # Telemetry data
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.airspeed = 0
        self.groundspeed = 0
        self.heading = 0
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.battery_voltage = 0
        self.battery_remaining = 0

        self.gimbal_pitch = 0
        self.gimbal_yaw = 0

        # Set default values for variables
        self.communication = communication  # Communication with other components
        self.connection = None

        # Connect to the vehicle
        self.connect_to_vehicle()

    def connect_to_vehicle(self, connection_string=ConnectionStrings.SITL, baudrate=115200):
        timeout = 10  # seconds

        try:
            print(f"Connecting to a vehicle on: {connection_string}")
            self.connection = mavutil.mavlink_connection(connection_string, baud=baudrate, autoreconnect=True,
                                                         timeout=timeout)

            print("Waiting for heartbeat...")
            if self.connection.wait_heartbeat(timeout=timeout):
                print("Connected")
                # Update telemetry data
                telemetry_thread = threading.Thread(target=self.update_telemetry_data)
                telemetry_thread.start()
            else:
                print("Connection failed")
        except Exception as e:
            print(f"Failed to connect: {e}")

    def update_telemetry_data(self):

        type_list = ['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'SYS_STATUS',
                     'GIMBAL_DEVICE_ATTITUDE_STATUS']

        while True:
            # Get the latest message from the vehicle
            msg = self.connection.recv_match(type=type_list)
            if msg is not None:
                # Update indicators
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.latitude = msg.lat / 1e7
                    self.longitude = msg.lon / 1e7
                    self.altitude = msg.relative_alt / 1e3
                if msg.get_type() == 'VFR_HUD':
                    self.airspeed = msg.airspeed
                    self.groundspeed = msg.groundspeed
                    self.heading = msg.heading
                    self.throttle = msg.throttle
                if msg.get_type() == 'ATTITUDE':
                    self.roll = msg.roll
                    self.pitch = msg.pitch
                    self.yaw = msg.yaw
                if msg.get_type() == 'SYS_STATUS':
                    self.battery_voltage = msg.voltage_battery
                    self.battery_remaining = msg.battery_remaining

                if msg.get_type() == 'GIMBAL_DEVICE_ATTITUDE_STATUS':
                    roll, self.gimbal_pitch, self.gimbal_yaw = self.quaternion_to_euler_angles(msg.q)

            time.sleep(0.02)

    def set_flight_mode(self, mode):
        self.connection.set_mode(mode)

    def arm(self):
        self.connection.arducopter_arm()

    def land(self):
        self.connection.set_mode('QLAND')

    def return_to_launch(self):
        """RTL"""
        self.connection.set_mode('QRTL')

    def takeoff(self, target_altitude):
        self.connection.arducopter_arm()

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, target_altitude)

    def move_to(self, lat, lng, speed=-1):
        lat = int(lat * 1e7)
        lng = int(lng * 1e7)
        alt = self.altitude
        # Send command to move to the specified latitude, longitude, and current altitude
        self.connection.mav.command_int_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            # “frame” = 0 or 3 for alt-above-sea-level, 6 for alt-above-home or 11 for alt-above-terrain
            dialect.MAV_CMD_DO_REPOSITION,
            0,  # Current
            0,  # Autocontinue
            speed,
            0, 0, 0,  # Params 2-4 (unused)
            lat,
            lng,
            alt
        )

    def set_roi(self, lat, lng, alt=0):
        lat = int(lat * 1e7)
        lng = int(lng * 1e7)
        self.connection.mav.command_int_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            # “frame” = 0 or 3 for alt-above-sea-level, 6 for alt-above-home or 11 for alt-above-terrain
            dialect.MAV_CMD_DO_SET_ROI_LOCATION,
            0,  # Current
            0,  # Autocontinue
            0, 0, 0, 0,  # Params 2-4 (unused)
            lat,
            lng,
            alt  # Altitude
        )

    def cancel_roi_mode(self):
        # Cancel the ROI mode.
        self.connection.mav.command_int_send(
            self.connection.target_system,
            self.connection.target_component,
            0,
            dialect.MAV_CMD_DO_SET_ROI_NONE,
            0, 0,
            0, 0, 0, 0,
            0, 0, 0)

    def set_gimbal_angle(self, pitch, yaw):
        # Send command to move to the specified latitude, longitude, and current altitude
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            pitch,
            yaw,
            0, 0,
            512, 0, 0)

    def set_gimbal_mode(self):
        # Send command to move to the specified latitude, longitude, and current altitude
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0, 0, 0, 0, 0,
            dialect.GIMBAL_DEVICE_FLAGS_RETRACT,
            0, 0)

    def take_gimbal_control(self):
        # Send the attitude command
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            0,
            -2,  # Take control
            0, 0, 0, 0, 0, 0
        )

        

    def quaternion_to_euler_angles(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).

        Args:
            q: A quaternion with attributes w, x, y, z.

        Returns:
            roll, pitch, and yaw in radians.
        """

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]))
        cosp = math.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


# TEST
if __name__ == '__main__':
    vehicle = Vehicle()
    oldtime = time.time()
    time.sleep(1)
    # vehicle.take_gimbal_control()
    # vehicle.set_gimbal_mode()
    vehicle.set_gimbal_angle(60, 0)
    # vehicle.set_roi(40, 40)
    while True:
        # print(time.time() - oldtime)
        oldtime = time.time()
        time.sleep(0.1)
