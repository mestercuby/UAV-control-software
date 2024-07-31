import threading
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import time
from otonom.rospyhand import rospyhand
from otonom.exploration import Exploration
from TopicServices import *
from otonom.tracker import *
from sensor_msgs.msg import *
from mavros.base import SENSOR_QOS



class Vehicle(rospyhand):
    mode = None
    armed = None

    #def __init__(self, node_name: str, rate: int,shared=None):
    def __init__(self, node_name: str):
        self.node_name = node_name
        #self.rate = rate
        self.armed = False
        self.gps = NavSatFix()
        #self.shared=shared
        self.TOPIC_STATE = TopicService("/mavros/state", State)
        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", CommandBool)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", SetMode)
        self.SERVICE_TAKEOFF = TopicService("/mavros/cmd/takeoff", CommandTOL)
        self.GPS_SUB = self.Subscriber( '/mavros/global_position/raw/fix',NavSatFix, self.global_position_cb, qos_profile=SENSOR_QOS)
        self.BATTERTY_SUB = self.Subscriber('/mavros/battery', BatteryState,  self.battery_cb, qos_profile=SENSOR_QOS)
        self.set_home_client = self.ServiceProxy('/mavros/cmd/set_home', CommandHome)

        self.set_home_position(self.lat, self.long, self.alt)

    def arm(self, status: bool):
        data = CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)            
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        success_value, result_value = result.success, result.result
        self.loginfo(f"Arm result: success={success_value}, result={result_value}")
        return result.success, result.result

    def takeoff(self,altitude: float):
        data = CommandTOLRequest()
        data.altitude = altitude
        self.SERVICE_TAKEOFF.set_data(data)
        result = self.service_caller(self.SERVICE_TAKEOFF, timeout=30)
        return result.success, result.result

    
    def set_mode(self,mode: str):
        data = SetModeRequest()
        data.custom_mode = str(mode)
        self.mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent  

    def global_position_cb(self, topic):
            self.loginfo("x %f: y: %f, z: %f" % (topic.latitude, topic.longitude, topic.altitude))
            #self.shared.update_coordinates(topic.latitude,topic.longitude)
            pass
    def battery_cb(self, topic):
            self.loginfo("V %s: I: %s" % (topic.voltage, topic.current))
            self.volt = topic.voltage
            self.curr = topic.current
            pass

    def set_home_position(self, latitude, longitude, altitude):
        self.wait_for_service('/mavros/cmd/set_home')
        
        try:
            request = CommandHomeRequest()
            request.current_gps = False  # Set manual position
            request.latitude = latitude
            request.longitude = longitude
            request.altitude = altitude
            
            response = self.set_home_client(request)
            
            if response.success:
                self.loginfo("Home position successfully set.")
            else:
                self.logwarn("Failed to set home position.")
        except self.ServiceException as e:
            self.logerr(f"Service call failed: {e}")
        initialpoint=(latitude,longitude,altitude)
        #self.shared.update_homep(initialpoint)

    def create_node(self):
        rospy.init_node("Vehicle")
        follow_me = Vehicle('Araç')  # pass coordinates to Arac class
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        rospy.signal_shutdown("Shutting down")
