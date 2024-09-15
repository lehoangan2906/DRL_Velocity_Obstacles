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
from geometry_msgs.msg import Pose, Twist
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


app = FaceAnalysis(providers = [('CUDAExecutionProvider')])
app.prepare(ctx_id = 0, det_size = (640, 640))

####################################################################################################


def find_Center(bbox):
    x1, y1, x2, y2 = bbox 
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    return [center_x, center_y]

def calculate_iou_vectorized(faces_boxes, human_boxes):
    pass

def top_k_pred():
    pass

def process_faces(img):
    """
    Detect faces and generate bounding boxes and embeddings using the InsightFace model.

    Args:
        img (np.ndarray): Input image from which faces are detected.

    Returns:
        faces_data (pd.DataFrame): A DataFrame containing the bounding boxes and embeddings of detected faces.
    """

    # Face detection and embedding extraction
    face_frame = img # Use the input image for face detection
    faces = app.get(face_frame) # Detect faces in the image using InsightFace

    # Create a DataFrame to store the bounding boxes of faces (x1, y1, x2, y2) and their embeddings
    faces_data = pd.DataFrame([face.bbox for face in faces], columns = ['x1', 'y1', 'x2', 'y2'])

    # Normalize and add embeddings to the DataFrame
    faces_data['embedding'] = [normalize(face.embedding) for face in faces]

    return faces_data


def normalize(emb):
    """
    Normalize the embedding vector.

    Args:
        emb (np.ndarray): Embedding vector to be normalized.

    Returns:
        np.ndarray: Normalized embedding vector.
    """

    emb = np.squeeze(emb)  # Remove single-dimensional entries from the shape of the array
    norm = np.linalg.norm(emb)  # Compute the L2 norm of the embedding vector
    return emb / norm   # Return the normalized embedding vector


def process_tracking(p, dt, im0s, paths, curr_frames, prev_frames, tracker_list, outputs,
                     faces_data, depth_frame, display_center, f_pixel, known_id,
                     collection_name, samples, client, save_crop, line_thickness, 
                     names, im, DEPTH_WIDTH, DEPTH_HEIGHT, save_vid, show_vid, 
                     hide_labels, hide_conf, hide_class, windows, seen, 
                     pre_velocities_cal, DELTA_T):
    
    """
    Process object tracking for the detected objects using the tracker.

    Args:
        p (list): List of detections from the model.
        dt (tuple): Timing profile for tracking and processing steps.
        im0s (list): Original images (before transformation).
        paths (list): List of image paths.
        curr_frames (list): List of current frames.
        prev_frames (list): List of previous frames for motion compensation.
        tracker_list (list): List of tracker instances.
        outputs (list): List to store tracker outputs.
        faces_data (pd.DataFrame): Data about detected faces, including embeddings.
        depth_frame: Depth frame from the RealSense camera.
        display_center (int): Horizontal center of the image.
        f_pixel (float): Focal length in pixels.
        known_id (dict): Dictionary of knwon person IDs,
        collection_name (str): Name of the Milvus collection for embeddings.
        samples (list): Embeddingsamples for comparison.
        client: Milvus client instance for querying.
        save_crop (bool): Whether to save cropped bounding boxes.
        line_thickness (int): Line thickness for bounding boxes.
        names (list): List of class names.
        im (np.ndarray): The image on which detections were made.
        DEPTH_WIDTH (int): Width of the depth image.
        DEPTH_HEIGHT (int): Height of the depth image.
        save_vid (bool): Whether to save the output video.
        show_vid (bool): Whether to show the video during processing.
        hide_labels (bool): Whether to hide the labels on the bounding boxes.
        hide_conf (bool): Whether to hide the confidence scores.
        hide_class (bool): Whether to hide the class names.
        windows (list): List of window names for video display.
        seen (int): Number of frames processed.
        pre_velocities_cal (pd.DataFrame): Previous velocity calculations for tracked objects.
        DELTA_T (float): Time difference between frames for velocity calculation.

    Returns:
        rs_dicts (list): List of dictionaries containing tracking information (IDs, bbox, depth, angle, velocities).
        curr_velocities_cal (pd.DataFrame): Current velocity calculations for the tracked objects.
    """

    rs_dicts = [] # Array to store information about tracked objects
    curr_velocities = [] # Array to store current velocities of objects

    for i, det in enumerate(p): # Iterate through detections per image
        seen += 1
        p, im0, _ = paths[i], im0s[i].copy(), 0 # Copy paths and original images
        curr_frames[i] = im0 # Update current frame

        # Prepare image for annotations
        imc = im0.copy() if save_crop else im0
        annotator = Annotator(im0, line_width=line_thickness, example = str(names))

        # Camera motion compensation, if required
        if hasattr(tracker_list[i], 'tracker') and hasattr(tracker_list[i].tracker, 'camera_update'):
            if prev_frames[i] is not None and curr_frames[i] is not None:
                tracker_list[i].tracker.camera_update(prev_frames[i], curr_frames[i])

        # If detection exist
        if det is not None and len(det):
            # Rescale bounding boxes to original image size
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

            # Pass detections to the tracker
            with dt[3]:
                outputs[i] = tracker_list[i].update(det.cpu(), im0)

            # If there are any tracking outputs
            if len(outputs[i]) > 0:
                # Store detection results in a DataFrame
                human_data = pd.DataFrame([output for output in outputs[i]],
                                          columns=['x1', 'y1', 'x2', 'y2', 'ID', 'cls', 'conf'])
                
                # Calculate IoU between human boxes and face boxes
                faces_boxes = faces_data[['x1', 'y1', 'x2', 'y2']].values
                human_boxes = human_data[['x1', 'y1', 'x2', 'y2']].values
                iou_matrix = calculate_iou_vectorized(faces_boxes, human_boxes)

                # Find closes matches based on IoU
                closest_B_indices = np.argmax(iou_matrix, axis=1)
                highest_ious = np.max(iou_matrix, axis=1)

                # Assign embeddings to the tracked objects
                human_data['embedding'] = None
                for l, idx in enumerate(closest_B_indices):
                    if highest_ious[l] > 0.5: # Assign only if the IoU is above a certain threshold
                        human_data.at[idx, 'embedding'] = faces_data.at[l, 'embedding']

                # Iterate over tracking outputs and perform further calculations
                for j, output in human_data.iterrows():
                    bbox = output[['x1', 'y1', 'x2', 'y2']].tolist()
                    id = output['ID']
                    cls = output['cls']
                    conf = output['conf']

                    # Handle embedding comparison using Milvus
                    if output['embedding'] is not None:
                        query_vectors = np.array([output['embedding']])

                        # Milvus query
                        param = {
                            'collection_name': collection_name,
                            'query_records': query_vectors,
                            'top_k': 1,
                            'params': {'nprobe': 16}
                        }
                        status, results = client.search(**param)

                        # Use thresholding on Milvus search results
                        th = 0.6
                        pred_name, score, ref = top_k_pred(0, 1, samples, results)
                        if score >= th:
                            pass # Logic for matching to known identity

                    # Calculate bounding box center and depth/angle
                    center = find_Center(bbox)
                    center[0] = max(0, min(center[0], DEPTH_WIDTH))
                    center[1] = max(0, min(center[1], DEPTH_HEIGHT))

                    d = abs(center[0] - display_center)
                    angle = np.arctan(d / f_pixel) # Calculate angle relative to camera
                    angle = angle if center[0] > display_center else -angle

                    depth = depth_frame.get_distance(int(center[0]), int(center[1]))  # Get depth value
                    real_x_position = depth * np.tan(angle)

                    # Calculate velocity if previous velocities are available
                    if not pre_velocities_cal.empty and not pre_velocities_cal.loc[pre_velocities_cal['ID'] == id].empty:
                        pre_velocity = pre_velocities_cal.loc[pre_velocities_cal['ID'] == id]
                        velocity_x = (pre_velocity['Real_x_position'].values[0] - real_x_position) / DELTA_T
                        velocity_y = (pre_velocity['Depth'].values[0] - depth) / DELTA_T
                        rs_dict = {'id': id, 'bbox': bbox, 'depth': depth, 'angle': angle, 'velocity': [velocity_x, velocity_y]}
                    else:
                        rs_dict = {'id': id, 'bbox': bbox, 'depth': depth, 'angle': angle, 'velocity': None}

                    rs_dicts.append(rs_dict) # Append results to the list
                    curr_velocities.append([id, real_x_position, depth])

            # Store current velocities
            curr_velocities_cal = pd.DataFrame(curr_velocities, columns=['ID', 'Real_x_position', 'Depth'])
        
        else:
            curr_velocities_cal = pd.DataFrame(columns=['ID', 'Real_x_position', 'Depth'])

        # Update the previous frames for motion compensation
        prev_frames[i] = curr_frames[i]

        # Display results
        if show_vid:
            if platform.system() == 'Linux' and p not in windows:
                windows.append(p)
                cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
                cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
            cv2.imshow(str(p), im0)
            if cv2.waitKey(1) == ord('q'):
                exit()

    return rs_dicts, curr_velocities_cal


def pre_transform(im, imgsz, model):
    """
    Pre-transform input images before passing them to the model for inference.

    Args:
        im (list of np.ndarray): List of images. Each image is expected to be in (h, w, 3) format.
        imgsz (tuple): Target image size (width, height) for inference.
        model (nn.Module): YOLOv5 model, used to check properties such as stride and format.

    Returns:
        list: A list of images after aopplying the transformations (e.g., resizing with letterboxing).
    """

    # Determine whether all images have the same shape
    same_shapes = len({x.shape for x in im}) == 1

    # Initialize the LetterBox transformation with the appropriate parameters
    letterbox = LetterBox(imgsz, auto=same_shapes and model.pot, stride=model.stride)

    # Apply the LetterBox transformation to each image in the list
    return [letterbox(image=x) for x in im]


def process_detection(dt, im0s, paths, model, device, imgsz, save_dir, is_seg, conf_thres, iou_thres, max_det, augment, visualize, classes, agnostic_nms):
    """
    Process the images for object detection using the YOLOv5 model.

    Args:
        dt (tuple): Timing profile.
        im0s (list of np.ndarray): Input images.
        paths (list of str): List of image paths.
        model (nn.Module): YOLOv5 model.
        device (torch.device): Device (CPU or GPU)
        imgsz (tuple): Image size for inference.
        save_dir (Path): Directory to save images.
        is_sqg (bool): Whether segmentation is enabled.
        conf_thres (float): Confidence threshold.
        iou_thres (float): IoU threshold for non-max suppression.
        max_det (int): Maximum number of detections per image.
        augment (bool): Whether to use augmentation.
        visualize (bool): Whether to visualize features.
        classes (list or None): Classes to filter by.
        agnostic_nms (bool): Use class-agnostic NMS.

    Returns:
        p (list): Processed detections for each image.
        im (torch.Tensor): Preprocessed image tensor ready for model inference.
    """
    
    with dt[0]: # Preprocessing
        # Check if the input is not already a torch Tensor
        not_tensor = not isinstance(im0s, torch.Tensor)

        if not_tensor:
            # Apply pre-transformation
            im = np.stack(pre_transform(im0s, imgsz, model))
            im = im[..., ::-1].transpose((0, 3, 1, 2)) # Convert BGR to RGB, BHWC to BCHW, (n, 3, h, w)
            im=np.ascontigousarray(im) # Ensure memory is contiguous
            im = torch.from_numpy(im) # Convert Numpy array to a Pytorch tensor

            im = im.to(device) # Move the tensor to the specified device (CPU or GPU)
            im = im.half() if model.fp16 else im.float() # Convert to FP16 or FP32 depending on model settings
            if not_tensor:
                im /= 255 # Normalize to [0.0, 1.0] if the input is not already a tensor
            
    # Inference
    with dt[1]:
        # If visualization is enabled, create the save directory
        visualize = (
            increment_path(save_dir / Path(paths).stem, mkdir=True)
            if visualize # And (not source_type.tensor)
            else False
        )
        # Perform inference using the model
        preds = model(im, augment=augment, visualize=visualize, embed=None)

    # Apply NMS
    with dt[2]:
        if is_seg:
            # For segmentation, handle masks
            masks = []
            p = non_max_suppression(preds[0], conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det, nm=32)
            proto = preds[1][-1]
        else:
            # Standard NMS without segmentation
            p = non_max_suppression(preds, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

    return p, im


def init_Milvus():
    pass

class TrackPedPublisher(Node):
    def __init__(self):
        super().__init__('track_ped_publisher')

        # Publisher for TrackedPersons (list of tracked objects)
        self.publisher_tracked_persons = self.create_publisher(TrackedPersons, '/track_ped', 10)

        # Publisher for individual TrackedPerson
        self.publisher_tracked_person = self.create_publisher(TrackedPerson, '/track_ped_person', 10)

        # Timer for calling the tracking callback at 30 FPS
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

        # Declare ROS2 parameters (can be configured via launch file or parameter server)
        self.declare_parameter('source', '0')  # Default to '0' (webcam/RealSense)
        self.declare_parameter('yolo_weights', str(WEIGHTS / 'yolov5m.pt'))
        self.declare_parameter('reid_weights', str(WEIGHTS / 'osnet_x0_25_msmt17.pt'))
        self.declare_parameter('tracking_method', 'strongsort')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('max_det', 1000)
        self.declare_parameter('show_vid', False)
        self.declare_parameter('save_vid', False)
        self.declare_parameter('device', '')

        # Get parameters from ROS2
        self.source = self.get_parameter('source').get_parameter_value().string_value
        self.yolo_weights = self.get_parameter('yolo_weights').get_parameter_value().string_value
        self.reid_weights = self.get_parameter('reid_weights').get_parameter_value().string_value
        self.tracking_method = self.get_parameter('tracking_method').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.iou_thres = self.get_parameter('iou_thres').get_parameter_value().double_value
        self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
        self.show_vid = self.get_parameter('show_vid').get_parameter_value().bool_value
        self.save_vid = self.get_parameter('save_vid').get_parameter_value().bool_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        # Handle the input source logic
        self.source = str(self.source)  # Ensure source is a string
        self.is_file = Path(self.source).suffix[1:] in (VID_FORMATS)  # Check if it's a file
        self.is_url = self.source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
        self.webcam = self.source.isnumeric() or self.source.endswith('.txt') or (self.is_url and not self.is_file)
        
        if self.is_url and self.is_file:
            self.source = check_file(self.source)  # If it's a URL and a file, check the file

        # Set up the device (GPU or CPU)
        self.device = select_device(self.device)

        # Initialize YOLOv5 model for object detection
        self.model = AutoBackend(self.yolo_weights, device=self.device, fp16=False)
        self.model.eval()  # Set model to evaluation mode
        self.names = self.model.names  # Class names from the YOLO model
        self.stride = self.model.stride  # Model stride
        self.imgsz = check_imgsz((640, 640), stride=self.stride)  # Inference image size

        # Warmup the model (to reduce initial inference latency)
        self.model.warmup(imgsz=(1, 3, *self.imgsz))

        # Initialize the tracker (StrongSort in this case)
        self.tracker_list = [create_tracker(self.tracking_method, None, self.reid_weights, self.device, False)]
        self.outputs = [None] * 1  # To hold tracking outputs
        self.prev_frames = [None] * 1  # Previous frames for camera motion compensation
        self.curr_frames = [None] * 1  # Current frames
        self.seen = 0  # Frame counter
        self.pre_velocities_cal = pd.DataFrame(columns=['ID', 'Real_x_position', 'Depth'])  # Velocity calculation storage

        # RealSense configuration for depth and color streams if using webcam
        if self.webcam:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            self.pipeline.start(config)
        else:
            # If using a file or other source, the pipeline would not be needed
            self.pipeline = None

        # Calculate focal length in pixels based on the camera's field of view
        self.display_center = 1280 // 2  # Horizontal center of the image
        self.f_pixel = (1280 * 0.5) / np.tan(69 * 0.5 * np.pi / 180)  # Horizontal focal length for angle calculation


        self.known_id = {}

        # Milvus or other database-related variables
        self.client, self.collection_name, self.samples = init_Milvus()  # Initialize Milvus database connection

        # Set up directories and save paths
        self.project = ROOT / 'runs' / 'track'
        self.name = 'exp'
        self.save_dir = increment_path(Path(self.project) / self.name, exist_ok=False)  # Increment run directory
        (self.save_dir / 'tracks').mkdir(parents=True, exist_ok=True)

    # Timer callback
    def timer_callback(self):
        if self.webcam:
            # Get frames from the RealSense camera
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                return # Return this frame if either depth or color frame is missing
            
            # Convert frames to NumPy arrays for processing
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            im0s = [color_image] # List of images to be processed
            paths = ['webcam'] # Using webcam

        else:
            # Use the source as the input path (for files or URLs)
            paths = [self.source]
            im0s = [cv2.imread(self.source)] # Replace with actual image loading logic for your source type 
            if im0s[0] is None:
                self.get_logger().error(f"Failed to load image from {self.source}")
                return # Skip processing this frame if the image fails to load

        # Process faces if needed 
        faces_data = process_faces(im0s[0])

        # Apply object detection
        p, im = process_detection(dt=None, im0s=im0s, paths=paths, model=self.model,
                                  device = self.device, imgsz=self.imgsz, save_dir=self.save_dir,
                                  is_seg=False, conf_thres=self.conf_thres, iou_thres=self.iou_thres,
                                  max_det=self.max_det, augment=False, visualize=False,
                                  classes=None, agnostic_nms=False)
        
        # Call process_tracking to track detected objects
        rs_dicts, self.pre_velocities_cal = process_tracking(p=p, dt=None, im0s=im0s, paths=paths, curr_frames=self.curr_frames,
                                                             prev_frames=self.prev_frames, tracker_list=self.tracker_list, outputs=self.outputs,
                                                             faces_data=faces_data, depth_frame=depth_frame if self.webcam else None, display_center = self.display_center,
                                                             f_pixel=self.f_pixel, known_id=self.known_id, collection_name=self.collection_name,
                                                             samples=self.samples, client=self.client, save_crop=False, line_thickness=self.line_thickness,
                                                             names=self.names, im=im0s[0], DEPTH_WIDTH=1280, DEPTH_HEIGHT=720, save_vid=self.save_vid,
                                                             show_vid=self.show_vid, hide_labels=self.hide_labels, hide_conf=self.hide_conf,
                                                             hide_class = self.hide_class, windows=[], seen=self.seen, pre_velocities_cal=self.pre_velocities_cal,
                                                             DELTA_T=1/30
                                                             )
        
        # Create the TrackedPersons message
        tracked_persons_msg = TrackedPersons()
        tracked_persons_msg.header = Header()
        tracked_persons_msg.header.stamp = self.get_clock().now().to_msg()
        tracked_persons_msg.header.frame_id = "camera_frame" # Adjust frame to your setup

        # Populate TrackedPersons message with tracked objects
        tracked_persons_msg.tracks = []

        for rs_dict in rs_dicts:
            tracked_person_msg = TrackedPerson()

            # Assign ID and bounding box
            tracked_person_msg.id = int(rs_dict['id'])
            tracked_person_msg.bbox_upper_left_x = rs_dict['bbox'][0]
            tracked_person_msg.bbox_upper_left_y = rs_dict['bbox'][1]
            tracked_person_msg.bbox_bottom_right_x = rs_dict['bbox'][2]
            tracked_person_msg.bbox_bottom_right_y = rs_dict['bbox'][3]

            # Assign depth and angle
            tracked_person_msg.depth = rs_dict['depth']
            tracked_person_msg.angle = rs_dict['angle']

            # Assign velocity if available
            if rs_dict['velocity']:
                tracked_person_msg.twist.linear.x = rs_dict['velocity'][0] # velocity in x direction
                tracked_person_msg.twist.linear.z = rs_dict['velocity'][1] # velocity in z direction
            else:
                tracked_person_msg.twist.linear.x = 0.0
                tracked_person_msg.twist.linera.z = 0.0

            
            # Calculate x and z coordinates of the pedestrian in the camera frame
            real_x = rs_dict['depth'] * np.tan(rs_dict['angle']) # Calculate x using trigonometry
            real_z = rs_dict['depth'] # z is the depth directly

            # Fill in the pose with real-world coordinates
            tracked_person_msg.pose = Pose()
            tracked_person_msg.pose.position.x = real_x
            tracked_person_msg.pose.position.y = 0.0
            tracked_person_msg.pose.position.z = real_z

            # Orientation can be set to default
            tracked_person_msg.pose.orientation.x = 0.0
            tracked_person_msg.pose.orientation.y = 0.0
            tracked_person_msg.pose.orientation.z = 0.0
            tracked_person_msg.pose.orientation.w = 1.0
            
            # Add individual tracked person to the list
            tracked_persons_msg.tracks.append(tracked_person_msg)

            # Optionally, publish individual tracked persons separately
            self.publisher_tracked_person.publish(tracked_person_msg)

        # Publish the TrackedPersons message (list of all tracked objects)
        self.publisher_tracked_persons.publish(tracked_persons_msg)