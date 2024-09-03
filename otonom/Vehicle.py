import threading
import time

from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

from otonom.Tracker import Tracker


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
        self.flight_mode = None

        self.gimbal_yaw_vehicle = 60
        self.gimbal_yaw_earth = 0
        self.gimbal_flag = -1

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

        type_list = ['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'SYS_STATUS', 'HEARTBEAT', 'GIMBAL_DEVICE_ATTITUDE_STATUS']

        while True:
            # Get the latest message from the vehicle
            msg = self.connection.recv_match(type=type_list)
            if msg is not None:
                # Update indicators
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.latitude = msg.lat / 1e7
                    self.longitude = msg.lon / 1e7
                    self.altitude = msg.relative_alt / 1000
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
                if msg.get_type() == 'HEARTBEAT':
                    self.flight_mode = mavutil.mode_string_v10(msg)

                if msg.get_type() == 'GIMBAL_DEVICE_ATTITUDE_STATUS':
                    print(msg)

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
        
        lat = int(float(lat * 1e7))
        lng = int(float(lng * 1e7))
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

    def set_roi(self, lat, lng, alt):
        # Send command to move to the specified latitude, longitude, and current altitude
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_DO_SET_ROI_LOCATION,
            0,
            0, 0, 0, 0,
            lat, lng, alt)

    def set_gimbal_angle(self, pitch, yaw):
        # Send command to move to the specified latitude, longitude, and current altitude
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            pitch,
            yaw,
            2, 2,
            2, 0, 0)

    def set_gimbal_mode(self):
        # Send the attitude command
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            dialect.MAV_CMD_DO_MOUNT_CONTROL,
            0,
            0, 0, 0, 0, 0, 0,
            dialect.MAV_MOUNT_MODE_MAVLINK_TARGETING  # Mode
        )

    def tracking_mission(self, target_id):
        tracker = Tracker(self, self.communication)
        print("tracking:", target_id)
        threading.Thread(target=tracker.track).start()


# TEST
if __name__ == '__main__':
    vehicle = Vehicle()
    oldtime = time.time()
    vehicle.set_gimbal_mode()
    time.sleep(1)
    vehicle.set_gimbal_angle(45, 30)
    while True:
        # print(time.time() - oldtime)
        oldtime = time.time()
        time.sleep(0.1)
