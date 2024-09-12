import socket
import threading
import argparse
import time
import os

from otonom.Vehicle import Vehicle
from server.server import VideoServerThread, HandleMessageThread
from image_processing.image_processing import image_process_main
from Communication import Communication
from simulation.subscribe_gz_image import ImageSubscriberThread
from otonom.PositionEstimator import PositionEstimator
from otonom.Tracker import Tracker


def main(args):
    print(args)
    fps = 30

    shared = Communication(fps)
    vehicle = Vehicle(shared)
    vehicle.connect_to_vehicle()
    vehicle.set_gimbal_angle(-60, 0)
    position_estimator = PositionEstimator(vehicle, shared)
    if args.isTest == 1:
        ImageSubscriberThread(shared).start()

    server = VideoServerThread(ip=args.ip, shared=shared)

    image_process = threading.Thread(target=image_process_main, args=(shared, args.isTest,position_estimator))
    image_process.start()
    server.start()

    #vehicle.tracking_mission(1)

    print("Waiting for wifi connection...")

    try:
        while server.client_socket == None:
            time.sleep(0.1)

        print("Wifi connection established!")

        handle_message_thread = HandleMessageThread(server.client_socket, shared)
        handle_message_thread.start()

        tracker = Tracker(vehicle, shared, position_estimator)
        threading.Thread(target=tracker.track).start()

        while True:

            # Handle new mission
            if shared.mission == "track":
                shared.track_mission = True
                shared.update_target(int(shared.argument))
            
            if shared.mission == "abort" or vehicle.flight_mode == 'QRTL':
                shared.track_mission = False

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted! Closing sockets and exiting...")
        if server.client_socket:
            server.client_socket.shutdown(socket.SHUT_RDWR)
        server.socket.shutdown(socket.SHUT_RDWR)
        os._exit(0)


parser = argparse.ArgumentParser(description='Process some arguments.')
parser.add_argument('--ip', type=str, default="127.0.0.1", help='ip address')
parser.add_argument('--isTest', type=bool, default=True, help='do you use simulation or real camera')

"""
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
