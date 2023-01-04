import pyrealsense2 as rs
import numpy as np
import cv2
try:
    from .detect_btn import *
except:
    from jeus_vision import *
from threading import Thread, Lock

import time
import os

SHARE_DIR = os.getcwd()
time_out = 1000000*(10**9)
def plot_one_box(x, img, color=None, label=None, line_thickness=3):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
class jeus_vision():
    def __init__(self) -> None:
        super().__init__()
        self.isStreaming = False
        
        self.lock_stream = Lock()
        self.lock_Val = Lock()
        self.rv = dict()
    def init_camera(self):

        # Declare pointcloud object, for calculating pointclouds and texture mappings
        self.pc = rs.pointcloud()
        # We want the points object to be persistent so we can display the last cloud when a frame drops
        self.points = rs.points()


        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        device = self.pipeline_profile.get_device()
        self.device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)


        self.config.enable_stream(rs.stream.depth,  1280,720, rs.format.z16, 5)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color,  1280,720, rs.format.bgr8, 5)


    def init_yolo(self,weight):
        weight_path =  os.path.join(SHARE_DIR,weight)
        # Initialize
        self.device =  select_device()
        # self.device = select_device(self.device)
        # Load model
        self.model = attempt_load(weight_path, device=self.device)  # load FP32 model
        # Get names and colors
        self.names =self.model.module.names if hasattr(self.model, 'module') else self.model.names  
        self.colors = dict()
        for _ in self.names.values():
            self.colors[_]= [random.randint(0, 255) for _ in range(3)]

    def stream_on_off(self):        
        if self.isStreaming:
            while self.lock_stream.locked():
                time.sleep(0.001)
            self.lock_stream.acquire()
            self.isStreaming = False
            self.lock_stream.release()
            self.thr.join()
            return False
        else:
            self.isStreaming = True
            self.thr = Thread(target=self.activate)
            self.thr.start()
            return True

            
    def get_Point(self, target):
        
        if self.isStreaming:                
            self.lock_Val.acquire()
            if target in self.rv.keys():
                buf = self.rv[target]
            else:
                buf = 'Not Detect'
            self.lock_Val.release()
            
        else:
            buf = "Is Not Streaming"
        return buf



    def activate(self):
        # Start streaming
        if hasattr(self, "pipeline") == False:
            self.init_camera()

        self.pipe_profile = self.pipeline.start(self.config)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        rv = None
        is_done = False
        start_time = time.time_ns()
        try_count = 0
        while self.isStreaming:
            self.lock_stream.acquire()
            
            frames = self.pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if depth_frame == None:
                continue
            img_color: np.ndarray = np.asanyarray(color_frame.get_data())
            img_depth: np.ndarray = np.asanyarray(depth_frame.get_data())

            # annotator = Annotator(img_color.copy(), line_width=2, example=str(self.names))

            result = detect(self.device, self.model, img_color, 
                            imgsz=[736, 1280], conf_thres=0.4, iou_thres=0.45, isVisualized=True)
            # result = detect(self.device, self.model, img_color, 
            #                 imgsz=list(img_color.shape[0:2]), conf_thres=0.4, iou_thres=0.45, isVisualized=True)
            current_time = time.time_ns() - start_time
            try_count += 1
            if result != None:

                self.pc.map_to(color_frame)
                self.points = self.pc.calculate(depth_frame)
                vtx = np.asanyarray(self.points.get_vertices())

                self.lock_Val.acquire()
                self.rv.clear()
                 # Write results
                for *xyxy, conf, cls in reversed(result):
                    c = int(cls)
                    cl = self.names[int(cls)]          
                    label = f'{self.names[c]} {conf:.2f}'                
                    plot_one_box(xyxy, img_color, label=label, color=self.colors[cl], line_thickness=2)            
                    # annotator.box_label(xyxy, label)
                    c1, c2 = (int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2))
                # Intrinsics & Extrinsics

                    i = 1280 * c2 + c1
                    # print ('point: ',[np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2])])
                    rv = (np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2]))
                    self.rv[cl] = rv
                self.lock_Val.release()

            # img = annotator.result()
            cv2.namedWindow("test", flags=cv2.WINDOW_NORMAL);
            cv2.imshow("test", img_color)
            cv2.waitKey(1)  
            self.lock_stream.release()
            time.sleep(0.002)
            # 1 millisecond
            # elif try_count > 20:
            #     is_done = True
            
        cv2.destroyAllWindows()
        self.pipeline.stop()
        

        

    def stop(self):
        self.pipeline.stop()
    
    
'''
pc = rs.pointcloud()
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()
img_color = np.asanyarray(color_frame.get_data())
img_depth = np.asanyarray(depth_frame.get_data())
pc.map_to(color)
points = pc.calculate(depth)
vtx = np.asanyarray(points.get_vertices())
tex = np.asanyarray(points.get_texture_coordinates())

npy_vtx = np.zeros((len(vtx), 3), float)
for i in range(len(vtx)):
    npy_vtx[i][0] = np.float(vtx[i][0])
    npy_vtx[i][1] = np.float(vtx[i][1])
    npy_vtx[i][2] = np.float(vtx[i][2])

npy_tex = np.zeros((len(tex), 3), float)
for i in range(len(tex)):
    npy_tex[i][0] = np.float(tex[i][0])
    npy_tex[i][1] = np.float(tex[i][1])
'''