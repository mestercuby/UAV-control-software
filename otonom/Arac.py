from rclpy.node import Node
import threading
import rclpy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import time
from rclpyhand import rclpyhand
from Exploration import Exploration
from TopicServices import *
from takip import *
from sensor_msgs.msg import *
from mavros.base import SENSOR_QOS



class Arac(RclHandler):
   ## SHARINGDE OLMASI GEREKEN DEĞİŞKENLER
    lat = None
    long = None
    altitude = None
    volt = None
    curr = None
    obj_detected = False ### GÖZLEM İÇİN
    height = None   ### GÖZLEM İÇİN (VE BELKİ TAKİP İÇİN)
    konum_x = 320.0
    konum_y = 240.0
    object_id = None
    takip_devam = True
    mission_finished = False


    ##
    obj_exploration_drone = None
    obj_track_drone = None
    mode = None
    armed = None


    def __init__(self, node_name: str, rate: int):
        super().__init__(node_name,rate)
        self.node_name = node_name
        self.rate = rate
        self.armed = False
        self.gps = NavSatFix()
        
        self.TOPIC_STATE = TopicService("/mavros/state", State)
        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", CommandBool)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", SetMode)
        self.SERVICE_TAKEOFF = TopicService("/mavros/cmd/takeoff", CommandTOL)
        self.GPS_SUB = self.create_subscription( '/mavros/global_position/raw/fix',NavSatFix, self.global_position_cb, qos_profile=SENSOR_QOS)
        self.BATTERTY_SUB = self.create_subscription('/mavros/battery', BatteryState,  self.battery_cb, qos_profile=SENSOR_QOS)
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        self.obj_exploration_drone = Exploration()
        self.obj_track_drone = takip()

        self.set_home_position(self.lat, self.long, self.alt)

    def arm(self, status: bool):
        data = CommandBool.Request()
        data.value = status
        self.SERVICE_ARM.set_data(data),            
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        success_value, result_value = result.success, result.result
        self.get_logger().info(f"Arm result: success={success_value}, result={result_value}")
        if result.result:
            self.get_logger().info(f"111 -- Arm result: success={success_value}, result={result_value}")
        else:
            self.get_logger().error("Arm service call failed")
        return None  

    def takeoff(self,altitude: float):
        data = CommandTOL.Request()
        data.altitude = altitude
        self.SERVICE_TAKEOFF.set_data(data)
        result = self.service_caller(self.SERVICE_TAKEOFF, timeout=30)
        return None

    
    def set_mode(self,mode: str):
        data = SetMode.Request()
        data.custom_mode = str(mode)
        self.mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return None  

    def global_position_cb(self, topic):
            #self.get_logger().info("x %f: y: %f, z: %f" % (topic.latitude, topic.longitude, topic.altitude))
            self.lat = topic.latitude
            self.long = topic.longitude
            self.altitude = topic.altitude
            pass
    def battery_cb(self, topic):
            #self.get_logger().info("V %s: I: %s" % (topic.voltage, topic.current))
            self.volt = topic.voltage
            self.curr = topic.current
            pass
    def start_mission(self):

        # ONAYI MAIN DE YER İSTASYONUNDAN ALACAK
        self.set_mode('GUIDED')
        # ARM OLMA DURUMUNU KONTROL
        if not self.armed:
            self.arm(True)

        # KOODİNATLARIN SEÇİLİ OLMASI GEREK
        # TAKEOFF İÇİN ONAY İSTE
        self.takeoff(self.height)

        self.obj_exploration_drone.start_explore()
        
        #mode değiştir
        ##################   OBJEYİ BULDUĞUNU KONTROL ET #######################
        if self.obj_detected :
            self.set_mode('GUIDED')
        ### GUIDED MODA GEÇTİĞİNİ KONTROL ET VE TRACKING BAŞLAT
            self.obj_track_drone.start_tracking()
        
        self.set_mode('RTL')

    def set_home_position(self, latitude, longitude, altitude):
        while not self.set_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set Home Position servisi bekleniyor...')
        
        request = CommandHome.Request()
        request.current_gps = False  # Manuel konum belirleyeceğiz
        request.latitude = latitude
        request.longitude = longitude
        request.altitude = altitude

        future = self.set_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Home pozisyonu başarılı bir şekilde ayarlandı.")
        else:
            self.get_logger().warn("Home pozisyonu ayarlanamadı.")

def main():
    rclpy.init('node', anonymous=True)
    
    follow_me = Arac('Araç', 400)  # pass coordinates to Arac class
    try:
        rclpy.spin(follow_me)
    except KeyboardInterrupt:
        pass
    follow_me.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
