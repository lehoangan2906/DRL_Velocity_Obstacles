import rclpy
from rclpy.node import Node
from track_ped_msgs.msg import TrackedPersons, TrackedPerson
from std_msgs.msg import Header

import os
import sys
import cv2
import math
import torch
import pickle
import logging
import platform
import argparse
import insightface
import numpy as np
import pandas as pd
import pyrealsense2 as rs


from time import time
from pathlib import Path
from find_distance import *
from utils.milvus_tool import *
from utils.process_box import *
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image


from ultralytics.utils.files import increment_path
from ultralytics.data import load_inference_source
from ultralytics.nn.autobackend import AutoBackend
from trackers.multi_tracker_zoo import create_tracker
from ultralytics.utils.torch_utils import select_device
from ultralytics.data.augment import LetterBox, classify_transforms
from ultralytics.utils.plotting import Annotator, colors, save_one_box
from ultralytics.data.utils import FORMATS_HELP_MSG, IMG_FORMATS, VID_FORMATS
from ultralytics.utils import DEFAULT_CFG, LOGGER, SETTINGS, callbacks, colorstr, ops
from ultralytics.utils.checks import check_file, check_imgsz, check_imshow, print_args, check_requirements
from ultralytics.utils.ops import Profile, non_max_suppression, scale_boxes, process_mask, process_mask_native


# limit the number of cpus used by high performance libraries
os.environ['OMP_NUM_THREADS'] = "1"
os.environ['MKL_NUM_THREADS'] = "1"
os.environ['NUMEXPR_NUM_THREADS'] = "1"
os.environ['OPENBLAS_NUM_THREADS'] = "1"
os.environ['VECLIB_MAXIMUM_THREADS'] = "1"


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 strongsort root directory
WEIGHTS = ROOT / 'weights'


if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'ultralytics') not in sys.path:
    sys.path.append(str(ROOT / 'ultralytics'))
if str(ROOT / 'trackers' / 'strongsort') not in sys.path:
    sys.path.append(str(ROOT / 'trackers' / 'strongsort')) 


sys.path.append(str(FILE.parents[2])) # add strong_sort ROOT to PATH
sys.path.append(str(FILE.parents[1]))


####################################################################################################


class TrackPedPublisher(Node):
    def __init__ (self):
        super().__init__('track_ped_publisher')

        # Publisher for TrackedPersons (list of tracked objects)
        self.publisher_tracked_persons = self.create_publisher(TrackedPersons, '/track_ped', 10)

        # Publisher for individual TrackedPerson
        self.publisher_tracked_person = self.create_publisher(TrackedPerson, '/track_ped_person', 10)

        self.timer = self.create_timer(1.0 / 30, self.timer_callback) # Assuming 30 FPS


    def process_tracking(p, dt, im0s, paths, curr_frames, 
                         prev_frames, tracker_list, outputs, 
                         faces_data, depth_frame, display_center, 
                         f_pixel, known_id, collection_name, samples, client, 
                         save_crop, line_thickness, names,
                         im, DEPTH_WIDTH, DEPTH_HEIGHT,
                         save_vid, show_vid, hide_labels,
                         hide_conf, hide_class, windows,
                         seen, pre_velocities_cal, DELTA_T):
        
        rs_dicts = [] # initialize the array to store dictionaries of pedestrians' informations
        curr_velocities = [] # initialize the array to store the current velocities of pedestrians

        # Process the detection process
        for i, det in enumerate(p): # deetection per frame
            seen += 1                                   # number of frames processed
            p, im0, _ = paths[i], im0s[i].copy(), 0     # dataset.count
            p = Path(p)                                 # to Path
            curr_frames[i] = im0                        # current frame

            imc = im0.copy() if save_crop else im0     # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))

            if hasattr(tracker_list[i], 'tracker') and hasattr(tracker_list[i].tracker, 'camera_update'):
                if prev_frames[i] is not None and curr_frames[i] is not None:   # Camera motion compensation
                    tracker_list[i].tracker.camera_update(prev_frames[i], curr_frames[i])

            if det is not None and len(det):

                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round() # Rescale boxes to im0 size

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # Detections per class

                # Pass detections to strongsort
                with dt[3]:
                    outputs[i] = tracker_list[i].update(det.cpu(), im0)

                # Draw boxes for visualization
                if len(outputs[i]) > 0:
                    human_data = pd.DataFrame([output for output in outputs[i]], columns=['x1', 'y1', 'x2', 'y2', 'ID', 'cls', 'conf'])

                    # Calculate IoU
                    faces_boxes = faces_data[['x1', 'y1', 'x2', 'y2']].values
                    human_boxes = human_data[['x1', 'y1', 'x2', 'y2']].values
                    iou_matrix = calculate_iou_vectorized(faces_boxes, human_boxes)

                    # Find the closest bounding box in group B for each box in A
                    closest_B_indices = np.argmax(iou_matrix, axis = 1)
                    highest_ious = np.max(iou_matrix, axis = 1)

                    # Add the array from A to B
                    human_data['embedding'] = None # Initialize the new column

                    for l, idx in enumerate(closest_B_indices):
                        human_data.at[idx, 'embedding'] = faces_data.at[l, 'embedding']

                    for j, (output) in human_data.iterrows(): 
                        bbox = output[['x1', 'y1', 'x2', 'y2']].tolist()
                        id = output['ID']
                        cls = output['cls']
                        conf = output['conf']

                        if output['embedding'] is not None:
                            query_vectors = np.array([output['embedding']])

                            top_k = 1
                            hyper_p = math.floor(top_k/2)

                            param = {
                                'collection_name': collection_name,
                                'query_records': query_vectors,
                                'top_k': top_k,
                                'params' : {'nprobe': 16}
                            }

                            status, reqults = client.search(**param)

                            th = 0.6
                            pred_name, score, ref = top_k_pred(0, top_k, samples, results)

                            if score > th:
                                pass
                        
                        center = find_Center(bbox)
                        center[0] = max(0, min(center[0], DEPTH_WIDTH))
                        center[1] = max(0, min(center[1], DEPTH_HEIGHT))

                        d = abs(center[0] - display_center)

                        angle = np.arctan(d/f_pixel)
                        angle = angle if center[0] > display_center else -angle
                        
                        depth = depth_frame.get_distance(int(center[0]), int(center[1]))
                        real_x_position = depth * np.tan(angle)

                        # Add bbox/seg to image
                        if save_vid or save_crop or show_vid: 
                            c = int(cls) # integer class
                            id = str(id) # integer id
                            label = None if hide_labels else (f'{id} {names[c]}' if hide_conf else (f'{id} {conf:.2f}' if hide_class else f'{id} {conf:.2f} Depth: {depth:.2f}m Angle: {angle:.2f}'))
                            color = colors(c, True)  # box color
                            annotator.box_label(bbox, label, color=color)
                        
                        if not pre_velocities_cal.empty and not pre_velocities_cal.loc[pre_velocities_cal['ID'] == id].empty:
                            pre_velocity = pre_velocities_cal.loc[pre_velocities_cal['ID'] == id]
                            velocity_x = (pre_velocity['Real_x_position'].values[0] - real_x_position) / DELTA_T
                            velocity_y = (pre_velocity['Depth'].values[0] - depth) / DELTA_T
                            rs_dict = {"id": id, "bbox": bbox, "depth": depth, "angle": angle, "velocity" : [velocity_x, velocity_y]}
                        else:
                            rs_dict = {"id": id, "bbox": bbox, "depth": depth, "angle": angle, "velocity": None}
                        
                        rs_dicts.append(rs_dict)
                        curr_velocities.append([id, real_x_position, depth])

                curr_velocities_cal = pd.DataFrame(curr_velocities, columns=['ID', 'Real_x_position', 'Depth'])
            else:
                curr_velocities_cal = pd.DataFrame(columns=['ID', 'Real_x_position', 'Depth'])
                pass

            prev_frames[i] = curr_frames[i] 

            # Stream results
            im0 = annotator.result()

            if show_vid:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO) # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)

                if cv2.waitKey(1) == ord('q'): # 1 millisecond
                    exit()

            return rs_dicts, curr_velocities_cal


    def timer_callback(self):
        # Get tracking data from process_tracking function
        rs_dicts, pre_velocity_cal = process_tracking(p, dt, im0s, paths, curr_frames, 
                                       prev_frames, tracker_list, outputs, 
                                       faces_data, depth_frame, display_center, 
                                       f_pixel, known_id, collection_name, samples, client,
                                       save_crop, line_thickness, names,
                                       im, DEPTH_WIDTH, DEPTH_HEIGHT,
                                       save_vid, show_vid, hide_labels,
                                       hide_conf, hide_class, windows,
                                       seen, pre_velocity_cal, DELTA_T)