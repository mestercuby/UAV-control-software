from mavros_msgs.msg import *
from mavros_msgs.srv import *
from otonom.rospyhand import RclHandler
from TopicServices import TopicService
import time
import math
from typing import List
import rclpy

class Tracker(rospyhand):

    kamera_fov_yatay = 63.0
    kamera_fov_dikey = 38.0
    
    piksel_x = 1920.0
    piksel_y = 1080.0

    #kameranın görüşü dışında kalan açı
    kamera_dikeyle_aci = 45.0
    #gimbal yatay açısı
    kamera_fov_yatay = 0.0

    # Sabit Açı anlık olarak kaç açı ile dönmelidir onu belirler
    sabit_aci = 3
    # Sabit X her bir çalışmasında kaç metre gideceğini belirtir \ KOŞULLARI VAR ONAA DİKKAT ETTT
    sabit_x = 0.5

    border_x1 = 0.495 * piksel_x
    border_x2 = 0.515 * piksel_x
    border_y1 = 0.49 * piksel_y
    border_y2 = 0.51* piksel_y


    def __init__(self,shared):
        super().__init__('takip',100)
        self.shared=shared
        self.target = PositionTarget()
        self.publisher = self.Publisher('/mavros/setpoint_raw/local',PositionTarget ,10)

    def start_track(self):
        while not self.shared.mission_finished:
            # YÖNLER HATALI OLABİLİR DEĞİŞTİR
            if 0 > self.shared.location[0] - (self.piksel_x // 2):
                yon_x = 1
            else:
                yon_x = -1
            if 0 > self.shared.location[1] - (self.piksel_y //2):
                yon_y = 1
            else:
                yon_y = -1
            self.publish_target_position(float(self.sabit_x * yon_y),0.0,(0.0174532925 * self.sabit_aci* yon_x))
            ### PUBLISH ETME HIZINI DEĞİŞTİR
            time.sleep(1)
        ###

    def publish_target_position(self,change_posx:float,change_posy:float,change_yaw:float):
        
        #########################################################################################################
        #################################  BURASI DEĞİŞECEK ARAŞTIR  ############################################
        #########################################################################################################


        self.target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.target.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ \
                           | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
                           | PositionTarget.IGNORE_YAW_RATE 
        

        ########################################################################################################
        #################################          HIZLAR DEĞİŞECEK    #########################################
        ########################################################################################################

        self.target.position.x = change_posx
        self.target.position.y = change_posy
        self.target.yaw = change_yaw
        self.target.position.z = 0.0  # desired altitude

        self.publisher.publish(self.target)
    
