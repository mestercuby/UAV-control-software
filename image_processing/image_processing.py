from ultralytics import YOLO
import cv2

from image_processing.tracker.ucmc import UCMCTrack
from image_processing.detector.mapper import Mapper
import numpy as np
import time
import math


def calculate_iou(box1, box2):
    """
    Calculate the Intersection over Union (IoU) of two bounding boxes.

    Parameters:
    box1 (tuple): (x1, y1, x2, y2) coordinates of the first box
    box2 (tuple): (x1, y1, x2, y2) coordinates of the second box

    Returns:
    float: IoU value
    """

    # Calculate the (x, y)-coordinates of the intersection rectangle
    xA = max(box1[0], box2[0])
    yA = max(box1[1], box2[1])
    xB = min(box1[2], box2[2])
    yB = min(box1[3], box2[3])

    # Compute the area of intersection rectangle
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    # Compute the area of both the prediction and ground-truth rectangles
    box1Area = (box1[2] - box1[0] + 1) * (box1[3] - box1[1] + 1)
    box2Area = (box2[2] - box2[0] + 1) * (box2[3] - box2[1] + 1)

    # Compute the intersection over union
    iou = interArea / float(box1Area + box2Area - interArea)

    return iou


def calculate_center(bbox):
    x_center = (bbox[0] + bbox[2]) / 2
    y_center = (bbox[1] + bbox[3]) / 2
    return x_center, y_center


def calculate_distance(center1, center2):
    return math.sqrt((center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2)


def merge_bboxes(group):
    x_min = min(det.bbox[0] for det in group)
    y_min = min(det.bbox[1] for det in group)
    x_max = max(det.bbox[2] for det in group)
    y_max = max(det.bbox[3] for det in group)
    id = min(det.track_id for det in group)
    conf = max(det.conf for det in group)

    #return [id, x_min, y_min, x_max, y_max]
    return Detection(0, x_min, y_min, x_max - x_min, y_max - y_min, [x_min, y_min, x_max, y_max], conf,track_id=id)


def grouping(current_dets, max_distance=100):
    groups = []
    visited = [False] * len(current_dets)

    for i in range(len(current_dets)):
        if visited[i]:
            continue

        group = [current_dets[i]]
        visited[i] = True

        if current_dets[i].track_id > 0:
            center1 = calculate_center(current_dets[i].bbox)
            for j in range(i + 1, len(current_dets)):
                if visited[j]:
                    continue

                center2 = calculate_center(current_dets[j].bbox)
                distance = calculate_distance(center1, center2)

                if distance <= max_distance:
                    group.append(current_dets[j])
                    visited[j] = True

        groups.append(group)

    merged_dets = [merge_bboxes(group) for group in groups]

    return merged_dets, groups


def check_surrounding(newdets, ldet, max_distance=300):
    if len(newdets) == 0:
        return None
    center1 = calculate_center(ldet.bbox)
    center2 = calculate_center(newdets[0].bbox)
    min_distance = calculate_distance(center1, center2)
    nearest_det_index = None
    for i in range(len(newdets)):
        ndet = newdets[i]
        center2 = calculate_center(ndet.bbox)
        distance = calculate_distance(center1, center2)
        if distance <= max_distance and distance <= min_distance:
            min_distance = distance
            nearest_det_index = i
    return nearest_det_index


# Detection:id,bb_left,bb_top,bb_width,bb_height, bbox,conf,det_class,track_id,y,R
class Detection:

    def __init__(self, id, bb_left=0, bb_top=0, bb_width=0, bb_height=0, bbox=[], conf=0, det_class=0,track_id=0):
        self.id = id
        self.bb_left = bb_left
        self.bb_top = bb_top
        self.bb_width = bb_width
        self.bb_height = bb_height
        self.bbox = bbox
        self.conf = conf
        self.det_class = det_class
        self.track_id = track_id
        self.y = np.zeros((2, 1))
        self.R = np.eye(4)

    def __str__(self):
        return 'd{}, bb_box:[{},{},{},{}], conf={:.2f}, class{}, uv:[{:.0f},{:.0f}], mapped to:[{:.1f},{:.1f}]'.format(
            self.id, self.bb_left, self.bb_top, self.bb_width, self.bb_height, self.conf, self.det_class,
            self.bb_left + self.bb_width / 2, self.bb_top + self.bb_height, self.y[0, 0], self.y[1, 0])

    def __repr__(self):
        return self.__str__()

    def to_dict(self):
        return {
            'id': int(self.id),
            'bb_left': int(self.bb_left),
            'bb_top': int(self.bb_top),
            'bb_width': int(self.bb_width),
            'bb_height': int(self.bb_height),
            'bbox' : [float(box) for box in self.bbox],
            'conf': float(self.conf),
            'det_class': int(self.det_class),
            'track_id': int(self.track_id),
            'y': self.y.tolist(),  # Convert numpy array to list
            'R': self.R.tolist()  # Convert numpy array to list
        }


# Detector
class Detector:
    def __init__(self):
        self.seq_length = 0
        self.gmc = None

    def load(self, cam_para_file):
        self.mapper = Mapper(cam_para_file, "MOT17")
        self.model = YOLO('image_processing/pretrained/yolov8s.pt')
        #self.model = YOLO('/home/master/Desktop/Nebula-image-processing/yolotrain/runs/detect/visdrone-s/weights/best.pt')

    def get_dets(self, img, conf_thresh=0, det_classes=[0]):

        dets = []

        # RGB（OpenCV BGR ）  
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # RTDETR 
        results = self.model(frame, imgsz=1088, verbose=False)

        det_id = 0
        for box in results[0].boxes:
            conf = box.conf.cpu().numpy()[0]
            bbox = box.xyxy.cpu().numpy()[0]

            cls_id = box.cls.cpu().numpy()[0]
            w = bbox[2] - bbox[0]
            h = bbox[3] - bbox[1]
            if w <= 10 and h <= 10 or cls_id not in det_classes or conf <= conf_thresh:
                continue

            #Detection
            det = Detection(det_id)
            det.bb_left = bbox[0]
            det.bb_top = bbox[1]
            det.bb_width = w
            det.bb_height = h
            det.conf = conf
            det.det_class = cls_id
            det.y, det.R = self.mapper.mapto([det.bb_left, det.bb_top, det.bb_width, det.bb_height])
            det.bbox = bbox
            det_id += 1

            dets.append(det)

        return dets


def image_process_main(shared, isTest, position_estimator=None):
    number = -1
    class_list = [0]
    mega_lost_dets = []
    dets_timers = {}
    dets_ids = {}
    threshold_time = 0.2
    id_counter = 0
    arg_a = 100.0
    arg_wx = 5
    arg_wy = 5
    arg_vmax = 10
    arg_cdt = 10.0
    arg_high_score = 0.5
    arg_conf_thresh = 0.01
    flag = False
    #video_path="/home/master/Downloads/footage.mp4"
    #video_path="/home/master/Desktop/UAV-control-software/image_processing/demo/demo.mp4"
    if isTest:
        time.sleep(1)
        fps = 30
        width = 1280
        height = 720
    else:
        #video_path="nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw(memory:NVMM), width=(int)960, height=(int)480, format=(string)I420 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink drop=1"
        #cap = cv2.VideoCapture(video_path, cv2.CAP_GSTREAMER)
        try:
            cap = cv2.VideoCapture(0)
        except:
            cap = cv2.VideoCapture(1)
        # fps
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


    #cv2.namedWindow("demo", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("demo", width, height)

    cam_para_path = "image_processing/demo/cam_para.txt"
    detector = Detector()
    detector.load(cam_para_path)

    tracker = UCMCTrack(arg_a, arg_a, arg_wx, arg_wy, arg_vmax, arg_cdt, fps, "MOT", arg_high_score, False, None)

    frame_time = 1 / fps

    frame_id = 1
    print("Fps: ", fps)
    print("Width: ", width)
    print("Height: ", height)
    new_dets = []
    current_dets = []
    groups = []
    merged = []
    while True:
        start_time = time.time()

        detections_list = [[], []]
        if isTest:
            ret = True
            frame_img = shared.camera_image
        else:
            ret, frame_img = cap.read()
        if not ret:
            break

        dets = detector.get_dets(frame_img, arg_conf_thresh, class_list)
        tracker.update(dets, frame_id)

        if frame_id > 2:

            flag = True

            lost_dets = [det for det in old_dets if det.track_id not in [det.track_id for det in dets]]
            new_dets = [det for det in dets if det.track_id not in [det.track_id for det in old_dets]]
            current_dets = [det for det in dets if det.track_id not in [det.track_id for det in new_dets]]

            for det in lost_dets:
                if det.track_id != 0:
                    mega_lost_dets.append((det, frame_id))
                    #print(f"detection lost:{det.track_id}")
            for ndet in new_dets:
                if ndet.track_id == 0:
                    new_dets.remove(ndet)
                #print(f"new detection:{ndet.track_id}")
            for ldet in mega_lost_dets:
                if frame_id - ldet[1] > 10:
                    mega_lost_dets.remove(ldet)
                    continue
                else:
                    for ndet in new_dets:
                        if calculate_iou(ldet[0].bbox, ndet.bbox) > 0.5:
                            #print(f"ldet:{ldet[0].track_id} ndet:{ndet.track_id}")
                            ndet.track_id = ldet[0].track_id
                            mega_lost_dets.remove(ldet)
                            new_dets.remove(ndet)

                            ####
                            current_dets.append(ndet)
                            flag = False
                            ####

                            break
                    if flag:
                        nearest_det_index = check_surrounding(new_dets, ldet[0])
                        if nearest_det_index is not None:
                            ndet = new_dets[nearest_det_index]
                            #print(f"nearest_det:{ndet.track_id}, lost:{ldet[0].track_id}")
                            ndet.track_id = ldet[0].track_id
                            mega_lost_dets.remove(ldet)
                            new_dets.remove(ndet)

            current_dets.extend(new_dets)
            merged, groups = grouping(current_dets)

        

        if len(merged) == 0:
            shared.update_target(None)

        for i in range(len(merged)):
            group = merged[i]
            id = group.track_id

            if id not in dets_timers:

                dets_timers[id] = time.time()
                continue

            elif time.time() - dets_timers[id] >= threshold_time:
                detections_list[0].append(group)
                detections_list[1].append(len(groups[i]))

        for i in range(len(detections_list[0])):
            group = detections_list[0][i]
            id = group.track_id
            x_min = group.bb_left
            y_min = group.bb_top
            x_max = group.bb_left + group.bb_width
            y_max = group.bb_top + group.bb_height

            #delete this only for debugging
            

            if id not in dets_ids:
                dets_ids[id] = id_counter
                id_counter += 1

            if id > 0:
                if detections_list[1][i] > 1:
                    cv2.rectangle(frame_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
                    cv2.putText(frame_img, str(dets_ids[id]), (int(x_min), int(y_min)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 0, 255), 2)
                else:
                    cv2.rectangle(frame_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                    cv2.putText(frame_img, str(dets_ids[id]), (int(x_min), int(y_min)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (255, 0, 0), 2)

        frame_id += 1
        
        final_dets = []
        for detection in detections_list[0]:
            #if position_estimator.get_position(detection) < max_track_distance:
            position, distance= position_estimator.get_position(detection)
            det=detection.to_dict()
            det['position'] = position
            det['distance'] = distance
            final_dets.append(det)
        shared.update_detections(final_dets, frame_img)

        elapsed_time = time.time() - start_time
        sleep_time = frame_time - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        old_dets = dets

    cap.release()
    cv2.destroyAllWindows()
