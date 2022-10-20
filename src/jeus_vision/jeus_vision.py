import pyrealsense2 as rs
import numpy as np
import cv2
from .detect_btn import *
import os

SHARE_DIR = os.getcwd()
time_out = 5*(10**9)
class jeus_vision():
    def __init__(self) -> None:
        super().__init__()

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


        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    def init_yolo(self,weight):
        self.device = ''
        weight_path =  os.path.join(SHARE_DIR,weight)
        # Initialize
        self.device = select_device(self.device)
        # Load model
        self.model = attempt_load(weight_path, device=self.device)  # load FP32 model
        # Get names and colors
        self.names =self.model.module.names if hasattr(self.model, 'module') else self.model.names  
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in  self.names]

    def activate(self, target):
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
        while is_done == False:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            img_color: np.ndarray = np.asanyarray(color_frame.get_data())
            img_depth: np.ndarray = np.asanyarray(depth_frame.get_data())
            
            
            result = detect(self.device, self.model, img_color, self.names, self.colors, target,
                            imgsz=list(img_color.shape[0:2]), conf_thres=0.4, iou_thres=0.45, isVisualized=False)
            # current_time = time.time_ns() - start_time
            try_count += 1
            if result != None:

                # Intrinsics & Extrinsics
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

                # Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
                depth_sensor = self.pipe_profile.get_device().first_depth_sensor()
                depth_scale = depth_sensor.get_depth_scale()

                # Map depth to color
                # depth_pixel = [500,300]   # Random pixel
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, result, depth_scale)

                color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
                color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
                print ('depth: ',color_point)
                print ('depth: ',color_pixel)

                self.pc.map_to(color_frame)
                self.points = self.pc.calculate(depth_frame)
                vtx = np.asanyarray(self.points.get_vertices())
                tex = np.asanyarray(self.points.get_texture_coordinates())
                i = 640*result[1]+result[0]
                print ('depth: ',[np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2])])
                rv = (np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2]))
                is_done = True
                
            # elif current_time > time_out:
            #     is_done = True
                
            elif try_count > 20:
                is_done = True
            

        self.pipeline.stop()
        return rv

    def activate_test(self, target):
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
        while is_done == False:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            img_color: np.ndarray = np.asanyarray(color_frame.get_data())
            img_depth: np.ndarray = np.asanyarray(depth_frame.get_data())
            
            
            result = detect(self.device, self.model, img_color, self.names, self.colors, target,
                            imgsz=list(img_color.shape[0:2]), conf_thres=0.4, iou_thres=0.45, isVisualized=True)
            # current_time = time.time_ns() - start_time
            try_count += 1
            if result != None:

                # Intrinsics & Extrinsics
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

                # Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
                depth_sensor = self.pipe_profile.get_device().first_depth_sensor()
                depth_scale = depth_sensor.get_depth_scale()

                # Map depth to color
                # depth_pixel = [500,300]   # Random pixel
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, result, depth_scale)

                color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
                color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
                print ('depth: ',color_point)
                print ('depth: ',color_pixel)

                self.pc.map_to(color_frame)
                self.points = self.pc.calculate(depth_frame)
                vtx = np.asanyarray(self.points.get_vertices())
                tex = np.asanyarray(self.points.get_texture_coordinates())
                i = 640*result[1]+result[0]
                print ('depth: ',[np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2])])
                rv = (np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2]))
                # is_done = True
                
            # elif current_time > time_out:
            #     is_done = True
                
            elif try_count > 500:
                is_done = True
            

        # self.pipeline.stop()
        

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