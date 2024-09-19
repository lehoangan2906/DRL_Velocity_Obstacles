import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] 
print(ROOT)
if str(ROOT / 'ultralytics') not in sys.path:
    sys.path.append(str(ROOT / 'ultralytics'))

from ultralytics import YOLO
# sys.path.append("/home/huynq600/Desktop/dummy_robot")
# import insightface
# from insightface.app import FaceAnalysis
# from insightface.data import get_image as ins_get_image
# from time import time,sleep
# app = FaceAnalysis(providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
# app.prepare(ctx_id=0, det_size=(640, 640))

models = YOLO('yolov8s.pt')
from time import time,sleep
start = time()
results = models.export(format='engine') 
result = models("tmp.mp4", stream = True, half = True, show = False, classes = 0)
# print(time() - start)

# for rs in result:
#     print(rs.boxes.xyxy)