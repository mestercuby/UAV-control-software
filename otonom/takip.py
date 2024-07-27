from mavros_msgs.msg import *
from mavros_msgs.srv import *
from rclpyhand import RclHandler
from TopicServices import TopicService
import time
import math
from typing import List
import rclpy

class takip(RclHandler):

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
    takip_devam = True
    mission_finished = False


    ##
    kamera_fov_yatay = 63.0
    kamera_fov_dikey = 38.0
    
    piksel_x = 640.0
    piksel_y = 480.0

    #kameranın görüşü dışında kalan açı
    kamera_dikeyle_aci = 45.0
    #gimbal yatay açısı
    kamera_fov_yatay = 0.0

    # Sabit Açı anlık olarak kaç açı ile dönmelidir onu belirler
    sabit_aci = 5
    # Sabit X her bir çalışmasında kaç metre gideceğini belirtir \ KOŞULLARI VAR ONAA DİKKAT ETTT
    sabit_x = 1

    border_x1 = 0.48 * piksel_x
    border_x2 = 0.52 * piksel_x
    border_y1 = 0.48 * piksel_y
    border_y2 = 0.52* piksel_y


    def __init__(self):
        super().__init__('takip',400)
        
        self.target = PositionTarget()
        self.publisher = self.create_publisher(PositionTarget,'/mavros/setpoint_raw/local', 10)

    def dongu(self):
        # YÖNLER HATALI OLABİLİR DEĞİŞTİR
        if 0 > self.konum_x - (self.piksel_x // 2):
            yon_x = 1
        else:
            yon_x = -1
        if 0 > self.konum_y - (self.piksel_y //2):
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
        self.target.yaw =change_yaw
        self.target.position.z = 0.0  # desired altitude

        self.publisher.publish(self.target)
    
    def start_tracking(self):
        while self.takip_devam:
            self.dongu()
            self.input= input("Enter Input e/h")
            if self.input != 'e':
                self.takip_devam = False
