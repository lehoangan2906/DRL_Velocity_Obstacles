import argparse
import cv2
import os
from time import time

# limit the number of cpus used by high performance libraries
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
import platform
import numpy as np
from pathlib import Path
import torch
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 strongsort root directory
WEIGHTS = ROOT / 'weights'

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'ultralytics') not in sys.path:
    sys.path.append(str(ROOT / 'ultralytics'))
if str(ROOT / 'trackers' / 'strongsort') not in sys.path:
    sys.path.append(str(ROOT / 'trackers' / 'strongsort'))  # add strong_sort ROOT to PATH

# ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
#################################################
sys.path.append(str(FILE.parents[2]))  # add strong_sort ROOT to PATH
sys.path.append(str(FILE.parents[1]))
import insightface
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image
import pyrealsense2 as rs
from find_distance import *
import pandas as pd
from milvus import Milvus, IndexType, MetricType, Status
import pickle
# '''
# , {
# 'device_id': 0,
# 'arena_extend_strategy': 'kNextPowerOfTwo',
# 'gpu_mem_limit': 4 * 1024 * 1024 * 1024,
# 'cudnn_conv_algo_search': 'EXHAUSTIVE',
# 'do_copy_in_default_stream': True,
# }
# '''

app = FaceAnalysis(providers=[('CUDAExecutionProvider')])
app.prepare(ctx_id=0, det_size=(640, 640))

################################################
import logging
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.data import load_inference_source
from ultralytics.data.augment import LetterBox, classify_transforms
from ultralytics.data.utils import FORMATS_HELP_MSG, IMG_FORMATS, VID_FORMATS
from ultralytics.utils import DEFAULT_CFG, LOGGER, SETTINGS, callbacks, colorstr, ops
from ultralytics.utils.checks import check_file, check_imgsz, check_imshow, print_args, check_requirements
from ultralytics.utils.files import increment_path
from ultralytics.utils.torch_utils import select_device
from ultralytics.utils.ops import Profile, non_max_suppression, scale_boxes, process_mask, \
    process_mask_native
from ultralytics.utils.plotting import Annotator, colors, save_one_box

from trackers.multi_tracker_zoo import create_tracker

def normalize(emb):
    emb = np.squeeze(emb)
    norm = np.linalg.norm(emb)
    return emb / norm

def top_k_pred(j,top_k,samples,results):
    names = []
    for i in range(top_k):
        names.append(samples[results[j][i].id]['name'])  
        # names.append(samples['folder'][results[j][i].id])   
    check = {}
    for name in names:
        check[name] = names.count(name)
    _max = 0.
    for key in check.keys():
        if check[key] > _max:
            _max = check[key]
            out_name = key
    
    score = []
    cnt = 0
    for i in range(top_k):
        if samples[results[j][i].id]['name'] == out_name:
            # distance = min(distance, float(results[j][i].distance)) #float(results[j][i].distance)
            score.append(results[j][i].distance)
            cnt += 1
    return out_name, score[0], None #samples['imagepath'][results[j][0].id]


def init_Milvus():
    client = Milvus(uri='tcp://localhost:19530')
    # client = Milvus(uri='tcp://172.21.100.254:19530')
    client.list_collections()

    client.drop_collection('data')

    # Create collection demo_collection if it dosen't exist.
    collection_name = 'data'

    status, ok = client.has_collection(collection_name)
    if not ok:
        param = {
            'collection_name': collection_name,
            'dimension': 512,
            'metric_type': MetricType.IP  # optional
        }
    client.create_collection(param)

    _, collection = client.get_collection_info(collection_name)
    print("Collection: ", collection)
    status, result = client.count_entities(collection_name)
    print("Result: ", result)

    pkl_path = "../sample.pkl"
    with open(pkl_path, 'rb') as f:
        samples = pickle.load(f)
    names = [sample['name'] for sample in samples]
    names = set(names)

    # embs = np.array([np.array(i['emb']) for i in samples])
    embs = np.array([sample['emb'] for sample in samples])
    print(len(embs))
    print(len(samples))
    #%% convert list to numpy array
    knownEmbedding = embs
    print(knownEmbedding.shape)

    # knownNamesId = [i['name'] for i in samples]
    knownNamesId = [sample['name'] for sample in samples]
    print('len Ids: ',len(knownNamesId))
    print('len Embs: ',knownEmbedding.shape[0])
    # insert true data into true_collection_
    status, ids = client.insert(collection_name=collection_name, records=knownEmbedding, ids=list(range(len(knownNamesId))))
    if not status.OK():
        print("Insert failed: {}".format(status))
    print(len(ids))
    print('Status: ',status)
    #%%
    client.flush([collection_name])
    # Get demo_collection row count
    status, result = client.count_entities(collection_name)
    print(result)
    print(status) 
    ivf_param = {'nlist': 1024}
    status = client.create_index(collection_name, IndexType.FLAT, ivf_param)

    # describe index, get information of index
    status, index = client.get_index_info(collection_name)
    
    return client, collection_name, samples

def calculate_iou_vectorized(A, B):
    # Compute the intersection coordinates
    x_left = np.maximum(A[:, None, 0], B[:, 0])
    y_top = np.maximum(A[:, None, 1], B[:, 1])
    x_right = np.minimum(A[:, None, 2], B[:, 2])
    y_bottom = np.minimum(A[:, None, 3], B[:, 3])

    # Compute intersection area
    intersection_area = np.maximum(x_right - x_left, 0) * np.maximum(y_bottom - y_top, 0)

    # Compute areas of bounding boxes
    area_A = (A[:, 2] - A[:, 0]) * (A[:, 3] - A[:, 1])
    area_B = (B[:, 2] - B[:, 0]) * (B[:, 3] - B[:, 1])

    # Compute union area
    union_area = (area_A[:, None] + area_B) - intersection_area

    # Compute IoU
    iou = intersection_area / union_area

    return iou

def check_inside(face_box, human_box):
    if human_box[0] <= face_box[0] and human_box[1] <= face_box[1] and human_box[2] >= face_box[2] and human_box[3] >= \
            face_box[3]:
        return True
    else:
        return False


def find_Center(box):
    return [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]

def pre_transform(im, imgsz, model):
    """
    Pre-transform input image before inference.

    Args:
        im (List(np.ndarray)): (N, 3, h, w) for tensor, [(h, w, 3) x N] for list.

    Returns:
        (list): A list of transformed images.
    """
    same_shapes = len({x.shape for x in im}) == 1
    letterbox = LetterBox(imgsz, auto=same_shapes and model.pt, stride=model.stride)
    return [letterbox(image=x) for x in im]

@torch.no_grad()
def run(
        source='0',
        yolo_weights=WEIGHTS / 'yolov5m.pt',  # model.pt path(s),
        reid_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt',  # model.pt path,
        tracking_method='strongsort',
        tracking_config=None,
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        show_vid=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        save_trajectories=False,  # save trajectories for each track
        save_vid=False,  # save confidences in --save-txt labels
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs' / 'track',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=2,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        hide_class=False,  # hide IDs
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
        retina_masks=False,
):
    id_dic = {}
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
    if is_url and is_file:
        source = check_file(source)  # download

    # Dataloader
    bs = 1
    dataset = load_inference_source(
            source=source,
            batch=bs,
            vid_stride=vid_stride,
            buffer=False,
        )
    
    source_type = dataset.source_type
    # Configure depth and color streams

    # Directories
    if not isinstance(yolo_weights, list):  # single yolo model
        exp_name = yolo_weights.stem
    elif type(yolo_weights) is list and len(yolo_weights) == 1:  # single models after --yolo_weights
        exp_name = Path(yolo_weights[0]).stem
    else:  # multiple models after --yolo_weights
        exp_name = 'ensemble'
    exp_name = name if name else exp_name + "_" + reid_weights.stem
    save_dir = increment_path(Path(project) / exp_name, exist_ok=exist_ok)  # increment run
    (save_dir / 'tracks' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    is_seg = '-seg' in str(yolo_weights)
    model = AutoBackend(weights=yolo_weights,
                        device=device,
                        dnn=dnn, 
                        fp16=half,
                        batch=bs,
                        fuse=True,
                        verbose=True)
    model.eval()
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_imgsz(imgsz, stride=stride)  # check image size

    vid_path, vid_writer, txt_path = [None] * bs, [None] * bs, [None] * bs
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    # Create as many strong sort instances as there are video sources
    tracker_list = []
    for i in range(bs):
        tracker = create_tracker(tracking_method, tracking_config, reid_weights, device, half)
        tracker_list.append(tracker, )
        if hasattr(tracker_list[i], 'model'):
            if hasattr(tracker_list[i].model, 'warmup'):
                tracker_list[i].model.warmup()
    outputs = [None] * bs
    
    # Run tracking
    # model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(device=device),
                                Profile(device=device),
                                Profile(device=device),
                                Profile())
    curr_frames, prev_frames = [None] * bs, [None] * bs
    frame_idx = 0 
    client, collection_name, samples = init_Milvus()

    for frame_idx, batch in enumerate(dataset):

        paths, im0s, s  = batch#path, im, im0s, vid_cap, s = batch

        face_frame = im0s[0].copy()
        faces = app.get(face_frame)
        faces_data = pd.DataFrame([face.bbox for face in faces], columns= ['x1', 'y1', 'x2', 'y2'])
        faces_data['embedding'] = [normalize(face.embedding) for face in faces]
        DEPTH_WIDTH, DEPTH_HEIGHT = face_frame.shape[1], face_frame.shape[0]
        display_center = DEPTH_WIDTH // 2

        # visualize = increment_path(save_dir / Path(path[0]).stem, mkdir=True) if visualize else False
        with dt[0]:
            not_tensor = not isinstance(im0s, torch.Tensor)
            if not_tensor:
                im = np.stack(pre_transform(im0s, imgsz, model))
                im = im[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW, (n, 3, h, w)
                im = np.ascontiguousarray(im)  # contiguous
                im = torch.from_numpy(im)

                im = im.to(device)
                im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
                if not_tensor:
                    im /= 255  # 0 - 255 to 0.0 - 1.0

        # Inference
        with dt[1]:
            visualize = (
                        increment_path(save_dir / Path(paths).stem, mkdir=True)
                        if visualize #and (not source_type.tensor)
                        else False
                        )
            preds = model(im, augment=augment, visualize=visualize, embed=None)

        # Apply NMS
        with dt[2]:
            if is_seg:
                masks = []
                p = non_max_suppression(preds[0], conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det, nm=32)
                proto = preds[1][-1]
            else:
                p = non_max_suppression(preds, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        # Process detections
        for i, det in enumerate(p):  # detections per image       
            seen += 1
            if webcam:  # bs >= 1
                p, im0, _ = paths[i], im0s[i].copy(), 0#dataset.count
                p = Path(p)  # to Path
                s += f'{i}: '
                txt_file_name = p.name
                save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
            else:
                p, im0, _ = paths[i], im0s.copy(), getattr(0, 'frame', 0)
                p = Path(p)  # to Path
                # video file
                if Path(source).suffix[1:] in (VID_FORMATS):#source.endswith(VID_FORMATS):
                    txt_file_name = p.stem
                    save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
                # folder with imgs
                else:
                    txt_file_name = p.parent.name  # get folder name containing current img
                    save_path = str(save_dir / p.parent.name)  # im.jpg, vid.mp4, ...
            curr_frames[i] = im0

            txt_path = str(save_dir / 'tracks' / txt_file_name)  # im.txt
            # s += '%gx%g ' % im.shape[2:]  # print string
            imc = im0.copy() if save_crop else im0  # for save_crop

            annotator = Annotator(im0[0], line_width=line_thickness, example=str(names))

            if hasattr(tracker_list[i], 'tracker') and hasattr(tracker_list[i].tracker, 'camera_update'):
                if prev_frames[i] is not None and curr_frames[i] is not None:  # camera motion compensation
                    tracker_list[i].tracker.camera_update(prev_frames[i], curr_frames[i])
            if det is not None and len(det):
                # if is_seg:
                #     shape = im0.shape
                #     # scale bbox first the crop masks
                #     if retina_masks:
                #         det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], shape).round()  # rescale boxes to im0 size
                #         masks.append(process_mask_native(proto[i], det[:, 6:], det[:, :4], im0.shape[:2]))  # HWC
                #     else:
                #         masks.append(process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True))  # HWC
                #         det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], shape).round()  # rescale boxes to im0 size
                # else:
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0[0].shape).round()  # rescale boxes to im0 size
                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    # s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # pass detections to strongsort
                with dt[3]:
                    outputs[i] = tracker_list[i].update(det.cpu(), im0)

                # draw boxes for visualization
                if len(outputs[i]) > 0:
                    #
                    # if is_seg:
                    #     # Mask plotting
                    #     annotator.masks(
                    #         masks[i],
                    #         colors=[colors(x, True) for x in det[:, 5]],
                    #         im_gpu=torch.as_tensor(im0, dtype=torch.float16).to(device).permute(2, 0, 1).flip(0).contiguous() /
                    #         255 if retina_masks else im[i]
                    #     )
                    tmp = outputs[i]
                    human_data = pd.DataFrame([output for output in tmp], columns=['x1', 'y1', 'x2', 'y2', 'ID', 'cls', 'conf'])
                    # Calculate IoU
                    faces_boxes = faces_data[['x1', 'y1', 'x2', 'y2']].values
                    human_boxes = human_data[['x1', 'y1', 'x2', 'y2']].values
                    iou_matrix = calculate_iou_vectorized(faces_boxes, human_boxes)
                    # Find the closest bounding box in group B for each box in A
                    closest_B_indices = np.argmax(iou_matrix, axis=1)
                    highest_ious = np.max(iou_matrix, axis=1)

                    # Add the array from A to B
                    human_data['embedding'] = None  # Initialize the new column
                    for i, idx in enumerate(closest_B_indices):
                        human_data.at[idx, 'embedding'] = faces_data.at[i, 'embedding']

                    for j, (output) in enumerate(outputs[i]):
                        
                    #     # ID = None
                        # print(output)
                        bbox = output[0:4]
                    #     #####################################
                        center = find_Center(bbox)
                        center[0] = max(0, min(center[0], DEPTH_WIDTH))
                        center[1] = max(0, min(center[1], DEPTH_HEIGHT))
                        d = abs(center[0] - display_center)

                        # print(bbox)
                        id = output[4]#find_id_matched(output[4],id_dic)  # ID_name if check is True else output[4]#check is True else output[4]
                        # print(id)
                        cls = output[5]
                        conf = output[6]

                        # print(output)
                        if save_txt:
                            # to MOT format
                            bbox_left = output[0]
                            bbox_top = output[1]
                            bbox_w = output[2] - output[0]
                            bbox_h = output[3] - output[1]
                            # Write MOT compliant results to file
                            with open(txt_path + '.txt', 'a') as f:
                                f.write(('%g ' + '%s ' +'%g ' * 5 + '\n') % (frame_idx + 1, id, bbox_left,  # MOT format
                                                            bbox_top, bbox_w, bbox_h, i))
                        """
                        ID of box: id from id
                        Box coordinate: (x1, y1, x2, y2) from bbox
                        Depth: depth of bbox from  depth
                        Angle: angle of bbox from center of the screen from a        
                        """    
                        if save_vid or save_crop or show_vid:  # Add bbox/seg to image
                            c = int(cls)  # integer class
                            id = str(id)  # integer id
                            label = None if hide_labels else (f'{id} {names[c]}' if hide_conf else \
                                                                (
                                                                    f'{id} {conf:.2f}' if hide_class else f'{id} {conf:.2f}'))  # (f'{id} {conf:.2f}' if hide_class else f'{id} {names[c]} {conf:.2f}'))
                            color = colors(c, True)
                            annotator.box_label(bbox, label, color=color)

                            if save_trajectories and tracking_method == 'strongsort':
                                q = output[7]
                                tracker_list[i].trajectory(im0, q, color=color)
                            if save_crop:
                                txt_file_name = txt_file_name if (isinstance(path, list) and len(path) > 1) else ''
                                save_one_box(np.array(bbox, dtype=np.int16), imc,
                                            file=save_dir / 'crops' / txt_file_name / names[
                                                c] / f'{id}' / f'{p.stem}.jpg', BGR=True)

            else:
                pass
                # tracker_list[i].tracker.pred_n_update_all_tracks()

            # Stream results
            im0 = annotator.result()
            if show_vid:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                if cv2.waitKey(1) == ord('q'):  # 1 millisecond
                    exit()

            # Save results (image with detections)
            if save_vid:
                if vid_path[i] != save_path:  # new video
                    vid_path[i] = save_path
                    if isinstance(vid_writer[i], cv2.VideoWriter):
                        vid_writer[i].release()  # release previous video writer
                    if vid_cap:  # video
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    else:  # stream
                        fps, w, h = 9, im0.shape[1], im0.shape[0]
                    save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                    vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                vid_writer[i].write(im0)
            prev_frames[i] = curr_frames[i]

        # Print total time (preprocessing + inference + NMS + tracking)
        LOGGER.info(
            f"{s}{'' if len(det) else '(no detections), '}{sum([dt.dt for dt in dt if hasattr(dt, 'dt')]) * 1E3:.1f}ms")

    # Print results
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(
        f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS, %.1fms {tracking_method} update per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_vid:
        s = f"\n{len(list((save_dir / 'tracks').glob('*.txt')))} tracks saved to {save_dir / 'tracks'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    # if update:
    #     strip_optimizer(yolo_weights)  # update model (to fix SourceChangeWarning)


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yolo-weights', nargs='+', type=Path, default=WEIGHTS / 'ultralyticss-seg.pt',
                        help='model.pt path(s)')
    parser.add_argument('--reid-weights', type=Path, default=WEIGHTS / 'osnet_x0_25_msmt17.pt')
    parser.add_argument('--tracking-method', type=str, default='bytetrack', help='strongsort, ocsort, bytetrack')
    parser.add_argument('--tracking-config', type=Path, default=None)
    parser.add_argument('--source', type=str, default='0', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--show-vid', action='store_true', help='display tracking video results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--save-trajectories', action='store_true', help='save trajectories for each track')
    parser.add_argument('--save-vid', action='store_true', help='save video tracking results')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    # class 0 is person, 1 is bycicle, 2 is car... 79 is oven
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs' / 'track', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=2, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--hide-class', default=False, action='store_true', help='hide IDs')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--vid-stride', type=int, default=1, help='video frame-rate stride')
    parser.add_argument('--retina-masks', action='store_true', help='whether to plot masks in native resolution')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    opt.tracking_config = ROOT / 'trackers' / opt.tracking_method / 'configs' / (opt.tracking_method + '.yaml')
    print_args(vars(opt))
    return opt


def main(opt):
    check_requirements(requirements=ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
    run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
