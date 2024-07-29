import threading
import argparse
import time
from server.server import VideoServerThread,HandleMessageThread
from image_processing.image_processing import image_process_main
from image_processing import distance
from otonom import Exploration,Arac

def main(args):
    shared=sharing()
    server = VideoServerThread(ip=args.ip ,shared=shared)
    server.start()
    
    image_process=threading.Thread(target=image_process_main,args=(shared,))
    image_process.start()

    while server.client_socket == None:
        continue
    handle_message_thread = HandleMessageThread(server.client_socket,shared)
    handle_message_thread.start()

    while True:
        time.sleep(0.1)
        mission,argument=shared.get_mission()
        if  mission=="getready":
            break

    
    vehicle = Arac('Araç', 400,shared)
    node_thread=threading.Thread(target=vehicle.create_node)
    node_thread.start()
    

    while True:
        time.sleep(0.1)
        mission,argument=shared.get_mission()
        if mission=="scan" :
            selectedpoints=argument
            break
    print(f"points {selectedpoints} selected for scan")
    explorer=Exploration()
    while True:
        time.sleep(0.1)
        mission,argument=shared.get_mission()
        if mission=="takeoff":
            break
    
    vehicle.set_mode('GUIDED')
    # ARM OLMA DURUMUNU KONTROL
    if not vehicle.armed:
        vehicle.arm(True)

        print(f"started takeoff to altitude {argument}")
        # TAKEOFF İÇİN ONAY İSTE
        vehicle.takeoff(argument)

        explore_thread=threading.Thread(target=vehicle.obj_exploration_drone.start_explore)
        explore_thread.start()
        #mode değiştir
        ##################   OBJEYİ BULDUĞUNU KONTROL ET #######################
        while True:
            time.sleep(0.1)
            mission,argument=shared.get_mission()
            if mission=="track":
                vehicle.set_mode('GUIDED')
                ### GUIDED MODA GEÇTİĞİNİ KONTROL ET VE TRACKING BAŞLAT
                threading.Thread(target=vehicle.obj_track_drone.start_tracking).start()
                break
            elif mission=="abort":
                explore_thread

        
        vehicle.set_mode('RTL')
    



    

parser = argparse.ArgumentParser(description='Process some arguments.')
parser.add_argument('--ip', type=str, default = "192.168.1.163", help='ip address')


""""
parser.add_argument('--cam_para', type=str, default = "/home/master/Desktop//Otonom/image_processing/demo/cam_para.txt", help='camera parameter file name')
parser.add_argument('--wx', type=float, default=5, help='wx')
parser.add_argument('--wy', type=float, default=5, help='wy')
parser.add_argument('--vmax', type=float, default=10, help='vmax')
parser.add_argument('--a', type=float, default=100.0, help='assignment threshold')
parser.add_argument('--cdt', type=float, default=10.0, help='coasted deletion time')
parser.add_argument('--high_score', type=float, default=0.5, help='high score threshold')
parser.add_argument('--conf_thresh', type=float, default=0.01, help='detection confidence threshold')
"""
args = parser.parse_args()

class sharing:
    def __init__(self):
        self.last_update_time=None
        self.error_msg=""
        #CV to server
        self.detections = None
        self.frame=None
        

        # CV to otonomi
        self.obj_detected = False ### GÖZLEM İÇİN
        self.location=(-1,-1)

        #server to main
        self.mission=None
        self.argument=None

        #otnomi to server
        self.mission_finished=True
        #otonomi to CV
        self.lat = None
        self.long = None
        self.lidar_height = None
        self.uav_tilt = None
        self.gimbal_tilt = None
        #otnomi internal
        self.wanted_height = None
        self.homep=None
        #server to CV
        self.track_target = None

    def update_detections(self,new_detections,new_frame):
        self.detections = new_detections
        self.frame = new_frame
        self.last_update_time=time.time()
    def get_detections(self):
        return self.detections, self.frame, self.last_update_time
    

    def update_CV_readings(self,new_lat,new_long,new_lidar_height,new_uav_tilt,new_gimbal_tilt):
        self.lat=new_lat
        self.long=new_long
        self.lidar_height = new_lidar_height
        self.uav_tilt = new_uav_tilt
        self.gimbal_tilt = new_gimbal_tilt
    def get_CV_readings(self):
        return self.lat , self.long , self.lidar_height ,self.uav_tilt, self.gimbal_tilt
    
    
    def update_mission(self,new_message):
        command=new_message.split(' ')[0]
        if command=="abort" and self.mission!="takeoff" :
            self.mission="abort" 

        elif command in ["scan","track","takeoff"]:
            argument=new_message.split(' ')[1]
            if command=="takeoff" and len(argument)!=2 :
                self.error_msg="wrong argument count for takeoff command"
                return
            elif len(argument)!=1 :
                self.error_msg=f"wrong argument count for {command} command"
                return
            argument=[float(a) for a in argument.split(',')]

            
            self.argument=argument
        else :
            self.argument=None 
        self.mission=command
    def get_mission(self):
        return self.mission , self.argument             

main(args)