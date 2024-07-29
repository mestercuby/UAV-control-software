import threading
import argparse
import time
from server.server import VideoServerThread
from image_processing.image_processing import image_process_main


def main(args):
    fps=30
    shared=sharing(fps)
    server = VideoServerThread(ip=args.ip ,shared=shared)
    image_process=threading.Thread(target=image_process_main,args=(shared,))

    server.start()
    image_process.start()



class sharing:
    def __init__(self,fps):
        self.detections = None
        self.frame=None
        self.last_update_time=None
        self.fps=fps
    def update_detections(self,new_detections,new_frame):
        self.detections = new_detections
        self.frame = new_frame
        self.last_update_time=time.time()

    def get_detections(self):
        return self.detections, self.frame, self.last_update_time
    


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

main(args)
