# import rs_test

# ts = rs_test.rs_test()
# ts.act()Skip to content


## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import time

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
import traceback

from jeus_armcontrol.jeus_log import *
from jeus_vision.models.experimental import attempt_load
from jeus_vision.utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from jeus_vision.utils.general import (LOGGER, apply_classifier, check_file, check_img_size, check_imshow, check_requirements,
                           check_suffix, colorstr, increment_path, non_max_suppression, print_args, scale_boxes,
                           strip_optimizer, xyxy2xywh)
from jeus_vision.utils.plots import Annotator, colors, save_one_box
from jeus_vision.utils.torch_utils import select_device, time_sync


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

def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)

def img_preprocess(img0, imgsz, half, stride, device):
    img = [letterbox(x, imgsz, stride=stride)[0] for x in img0]
    # Stack
    img = np.stack(img, 0)
    # Convert
    img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    return img

def view(img):
    # data, t2, t1, img = detected_data.get()
    # cv2.putText(img, "FPS : %0.2f" % f, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
    #print("yolo img:", img.shape)

    cv2.imshow('VIEW', img)
    cv2.waitKey(1)

def detect(device, model, img_raw: np.ndarray, names, colors,target,
           imgsz=[448, 640], conf_thres=0.01, iou_thres=0.25, classes=None, agnostic_nms=False, isVisualized = True, verbose = False):

    img_raw = np.expand_dims(img_raw, axis=0)

    half = device.type != 'cpu'  # half precision only supported on CUDA
    if half:
        model.half()  # to FP16

    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once

    img0 = img_raw
    img = img_preprocess(img0, imgsz, half, stride, device)

    # Inference
    pred = model(img)[0]

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
    # gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

    # Visualization
    det = pred[0]
    im0 = img0[0].copy()
    # if isVisualized: im0 = img0[0].copy()
    #print("im0_shpape:", im0.shape)
    send_data = None
    
    if len(det):
        # Rescale boxes from img_size to im0 size
        det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], im0.shape).round()

        # Write results
        for *xyxy, conf, cls in reversed(det):
            c = int(cls)
            cl = names[int(cls)]
            if cl == target:
                center = (int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2))
                c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                send_data = center
            if isVisualized:
                label = f'{names[c]} {conf:.2f}'
                plot_one_box(xyxy, im0, label=label, color=colors[c], line_thickness=1)            
                #annotator.box_label(xyxy, label, color=colors(c, True))
    else:
        send_data = None

    # Print time (inference + NMS)
    if verbose:
        #print(f'Done. ({t2 - t1:.3f}s)')
        print(f"{target} Center : {send_data}")

    if isVisualized: cv2.imwrite('./result/btn_{0}'.format(time.time()), im0)
    
    return send_data

weight_path =  "./btn_221203/best.pt"
# Initialize
device =  select_device()
# self.device = select_device(self.device)
# Load model
model = attempt_load(weight_path, device=device)  # load FP32 model
names = model.module.names if hasattr(model, 'module') else model.names
# Get names and colors
colors = [[random.randint(0, 255) for _ in range(3)] for _ in  names]


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 1280,720, rs.format.z16, 5)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color,1280,720, rs.format.bgr8, 5)

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# if device_product_line == 'L500':
#     config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
# else:
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# Start streaming
align_to = rs.stream.color
align = rs.align(align_to)
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        
        aligned_frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        detect(device, model, color_image, names, colors, 'btn_1',
                            imgsz=list(color_image.shape[0:2]), conf_thres=0.4, iou_thres=0.45, isVisualized=True)
            
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape




 

            
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
