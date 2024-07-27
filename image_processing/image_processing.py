from ultralytics import YOLO
import cv2

from image_processing.tracker.ucmc import UCMCTrack
from image_processing.detector.mapper import Mapper
import numpy as np
import time


    

# Detection:id,bb_left,bb_top,bb_width,bb_height,conf,det_class
class Detection:

    def __init__(self, id, bb_left = 0, bb_top = 0, bb_width = 0, bb_height = 0, conf = 0, det_class = 0):
        self.id = id
        self.bb_left = bb_left
        self.bb_top = bb_top
        self.bb_width = bb_width
        self.bb_height = bb_height
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
        self.model = YOLO('/home/master/Desktop/Otonom/image_processing/pretrained/yolov8x.pt')
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
            det_id += 1

              
            dets.append(det)

        return dets
    

def image_process_main(shared):
    number = -1
    class_list = [0,2,5,7]
    valid_dets={}
    threshold_time=0.5
    arg_a=100.0
    arg_wx=5
    arg_wy=5
    arg_vmax=10
    arg_cdt=10.0
    arg_high_score=0.5
    arg_conf_thresh=0.01
    
    cap = cv2.VideoCapture(0)
    # fps
    fps = cap.get(cv2.CAP_PROP_FPS)

   

    
    

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    #video_out = cv2.VideoWriter('/home/master/Desktop/ucmctest/output/output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))  

    #cv2.namedWindow("demo", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("demo", width, height)

    cam_para_path="/home/master/Desktop//Otonom/image_processing/demo/cam_para.txt"
    detector = Detector()
    detector.load(cam_para_path)

    
    tracker = UCMCTrack(arg_a, arg_a, arg_wx, arg_wy, arg_vmax, arg_cdt, fps, "MOT", arg_high_score,False,None)

    frame_time = 1 / fps

    frame_id = 1



    while True:
        start_time=time.time()
        detections_list=[]
        ret, frame_img = cap.read()
        if not ret:  
            break
    
        dets = detector.get_dets(frame_img,arg_conf_thresh,class_list)
        tracker.update(dets,frame_id)
        
        for det in dets :
            if det.track_id not in valid_dets:
                 valid_dets[det.track_id]=time.time()
            elif time.time()- valid_dets[det.track_id] >=threshold_time :
                detections_list.append(det.to_dict())
        
        shared.update_detections(detections_list,frame_img)


        elapsed_time= time.time() -start_time
        sleep_time= frame_time - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        
        for det in dets:
            # 画出检测框
            if det.track_id ==number:
                cv2.rectangle(frame_img, (int(det.bb_left), int(det.bb_top)), (int(det.bb_left+det.bb_width), int(det.bb_top+det.bb_height)), (255, 0, 0), 2)
                # 画出检测框的id
                cv2.putText(frame_img, str(det.track_id), (int(det.bb_left), int(det.bb_top)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif det.track_id > 0:
                cv2.rectangle(frame_img, (int(det.bb_left), int(det.bb_top)), (int(det.bb_left+det.bb_width), int(det.bb_top+det.bb_height)), (0, 255, 0), 2)
                # 画出检测框的id
                cv2.putText(frame_img, str(det.track_id), (int(det.bb_left), int(det.bb_top)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        frame_id += 1


        # 显示当前帧
        cv2.imshow("demo", frame_img)
        if cv2.waitKey(1) & 0xFF== ord('q'):
            break
        

        



        #video_out.write(frame_img)
    
    cap.release()
    #video_out.release()
    cv2.destroyAllWindows()




