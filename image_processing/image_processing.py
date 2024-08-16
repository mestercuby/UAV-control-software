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
    return (x_center, y_center)

def calculate_distance(center1, center2):
    return math.sqrt((center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2)

def merge_bboxes(group):
    x_min = min(det.bbox[0] for det in group)
    y_min = min(det.bbox[1] for det in group)
    x_max = max(det.bbox[2] for det in group)
    y_max = max(det.bbox[3] for det in group)
    id = min(det.track_id for det in group)
    return [id, x_min, y_min, x_max, y_max]

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
    
    merged_bboxes = [merge_bboxes(group) for group in groups]
    
    return merged_bboxes

# Detection:id,bb_left,bb_top,bb_width,bb_height,conf,det_class
class Detection:

    def __init__(self, id, bb_left = 0, bb_top = 0, bb_width = 0, bb_height = 0, conf = 0, det_class = 0):
        self.id = id
        self.bb_left = bb_left
        self.bb_top = bb_top
        self.bb_width = bb_width
        self.bb_height = bb_height
        self.bbox=[]
        self.conf = conf
        self.det_class = det_class
        self.track_id = 0
        self.y = np.zeros((2, 1))
        self.R = np.eye(4)


    def __str__(self):
        return 'd{}, bb_box:[{},{},{},{}], conf={:.2f}, class{}, uv:[{:.0f},{:.0f}], mapped to:[{:.1f},{:.1f}]'.format(
            self.id, self.bb_left, self.bb_top, self.bb_width, self.bb_height, self.conf, self.det_class,
            self.bb_left+self.bb_width/2,self.bb_top+self.bb_height,self.y[0,0],self.y[1,0])

    def __repr__(self):
        return self.__str__()
    
    def to_dict(self):
        return {
            'id': int(self.id),
            'bb_left': int(self.bb_left),
            'bb_top': int(self.bb_top),
            'bb_width': int(self.bb_width),
            'bb_height': int(self.bb_height),
            'conf': float(self.conf),
            'det_class': int(self.det_class),
            'track_id': int(self.track_id),
            'y': self.y.tolist(),  # Convert numpy array to list
            'R': self.R.tolist()   # Convert numpy array to list
        }



# Detector
class Detector:
    def __init__(self):
        self.seq_length = 0
        self.gmc = None
        

    def load(self,cam_para_file):
        self.mapper = Mapper(cam_para_file,"MOT17")
        self.model = YOLO('/home/master/Desktop/UAV-control-software/image_processing/pretrained/yolov8x.pt')
        #self.model = YOLO('/home/master/Desktop/Nebula-image-processing/yolotrain/runs/detect/visdrone-s/weights/best.pt')


    def get_dets(self, img,conf_thresh = 0,det_classes = [0]):
        
        dets = []

        # RGB（OpenCV BGR ）  
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  

        # RTDETR 
        results = self.model(frame,imgsz = 1088)

        det_id = 0
        for box in results[0].boxes:
            conf = box.conf.cpu().numpy()[0]
            bbox = box.xyxy.cpu().numpy()[0]
            
            cls_id  = box.cls.cpu().numpy()[0]
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
            det.y,det.R = self.mapper.mapto([det.bb_left,det.bb_top,det.bb_width,det.bb_height])
            det.bbox=bbox
            det_id += 1

              
            dets.append(det)

        return dets
    

def image_process_main(shared):
    number = -1
    class_list = [0,2,5,7]
    mega_lost_dets=[]
    threshold_time=0.5
    arg_a=100.0
    arg_wx=5
    arg_wy=5
    arg_vmax=10
    arg_cdt=10.0
    arg_high_score=0.5
    arg_conf_thresh=0.01
    #video_path="/home/master/Downloads/footage.mp4"
    #video_path="/home/master/Desktop/UAV-control-software/image_processing/demo/demo.mp4"
    cap = cv2.VideoCapture("/home/master/Desktop/UAV-control-software/son.mp4")
    # fps
    fps = cap.get(cv2.CAP_PROP_FPS)

   
    print("hi")
    
    

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    video_out = cv2.VideoWriter('/home/master/Desktop/son1.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))  

    #cv2.namedWindow("demo", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("demo", width, height)

    cam_para_path="/home/master/Desktop/UAV-control-software/image_processing/demo/cam_para.txt"
    detector = Detector()
    detector.load(cam_para_path)

    
    tracker = UCMCTrack(arg_a, arg_a, arg_wx, arg_wy, arg_vmax, arg_cdt, fps, "MOT", arg_high_score,False,None)

    frame_time = 1 / fps

    frame_id = 1
    print(fps)
    print(frame_time)
    new_dets = []
    current_dets = []
    groups = []
    while True:
        start_time=time.time()
        
        detections_list=[]
        ret, frame_img = cap.read()
        if not ret:  
            break
        
        dets = detector.get_dets(frame_img,arg_conf_thresh,class_list)
        tracker.update(dets,frame_id)
        
        if frame_id > 2:

            lost_dets = [det for det in old_dets if det.track_id not in [det.track_id for det in dets]]
            new_dets = [det for det in dets if det.track_id not in [det.track_id for det in old_dets]]
            current_dets = [det for det in dets if det.track_id not in [det.track_id for det in new_dets]]

            for det in lost_dets:
                mega_lost_dets.append((det,frame_id))
                print(f"detection lost:{det.track_id}")
            for ndet in new_dets:
                print(f"new detection:{ndet.track_id}")
            for ldet in mega_lost_dets:
                if frame_id - ldet[1] > 5: 
                    mega_lost_dets.remove(ldet)
                else:
                    for ndet in new_dets:
                        if calculate_iou(ldet[0].bbox,ndet.bbox) > 0.5:
                            print(f"ldet:{ldet[0].track_id} ndet:{ndet.track_id}")
                            ndet.track_id = ldet[0].track_id
                            mega_lost_dets.remove(ldet)
                            
                            break
           
            current_dets.extend(new_dets)
            groups = grouping(current_dets)
          
                        
        '''     
        elif time.time()- valid_dets[det.track_id] >=threshold_time :
                detections_list.append(det.to_dict())
        #shared.update_detections(detections_list,frame_img)
        '''
        
        elapsed_time= time.time() -start_time
        sleep_time= frame_time - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        """
        for det in new_dets:
            print(det.track_id)
            # 画出检测框
            if det.track_id > 0:
                cv2.rectangle(frame_img, (int(det.bb_left), int(det.bb_top)), (int(det.bb_left+det.bb_width), int(det.bb_top+det.bb_height)), (0, 255, 0), 2)
                # 画出检测框的id
                cv2.putText(frame_img, str(det.track_id), (int(det.bb_left), int(det.bb_top)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        for det in current_dets:
            print(det.track_id)
            # 画出检测框
            if det.track_id > 0:
                cv2.rectangle(frame_img, (int(det.bb_left), int(det.bb_top)), (int(det.bb_left+det.bb_width), int(det.bb_top+det.bb_height)), (0, 255, 0), 2)
                # 画出检测框的id
                cv2.putText(frame_img, str(det.track_id), (int(det.bb_left), int(det.bb_top)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        """

 

        for group in groups:
            id = group[0]
            x_min = group[1]
            y_min = group[2]
            x_max = group[3]
            y_max = group[4]

            if id > 0:
                cv2.rectangle(frame_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                cv2.putText(frame_img, str(id), (int(x_min), int(y_min)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

  

        frame_id += 1
        

        cv2.imshow("demo", frame_img)
        if cv2.waitKey(1) & 0xFF== ord('q'):
            break
        

        


        old_dets = dets
        video_out.write(frame_img)
    
    cap.release()
    video_out.release()
    cv2.destroyAllWindows()




