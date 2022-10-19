
import time

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
import traceback

from jeus_armcontrol.jeus_log import *
from models.experimental import attempt_load
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, apply_classifier, check_file, check_img_size, check_imshow, check_requirements,
                           check_suffix, colorstr, increment_path, non_max_suppression, print_args, scale_coords,
                           strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import load_classifier, select_device, time_sync

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
    gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

    # Visualization
    det = pred[0]
    if isVisualized: im0 = img0[0].copy()
    #print("im0_shpape:", im0.shape)
    send_data = None
    
    if len(det):
        # Rescale boxes from img_size to im0 size
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

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

    if isVisualized: view(im0)

    return send_data

def yolo2pixel(center_x,center_y,img_x = 640,img_y=480):
    pixel_x = center_x* img_x
    pixel_y = center_y* img_y
    return pixel_x, pixel_y