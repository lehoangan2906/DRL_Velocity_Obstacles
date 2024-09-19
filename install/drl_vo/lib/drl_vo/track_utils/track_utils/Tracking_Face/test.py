# import the opencv library
import cv2
import cv2
import numpy as np
import insightface
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image
from find_distance import *
import pandas as pd

df = pd.read_pickle('database.pkl')
app = FaceAnalysis(providers=['CUDAExecutionProvider'])
app.prepare(ctx_id=0, det_size=(640, 640))
# define a video capture object
vid = cv2.VideoCapture(r'F:\PROJECT\KHOA_LUAN_MTMC\test\1_filename.mp4')
# so, convert them from float to integer.
frame_width = int(vid.get(3))
frame_height = int(vid.get(4))
size = (frame_width, frame_height)
df = pd.read_pickle('database.pkl')
result = cv2.VideoWriter(str(1) + 'filename.mp4',
                         cv2.VideoWriter_fourcc(*'mp4v'),
                         12, size)
while (True):

    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    faces = app.get(frame)
    if len(faces) !=0:
        current_embedding =  faces[0].embedding

        def findDistance(row):
            database_embedding = row['embedding']
            distance = findEuclideanDistance(current_embedding, database_embedding)
            return distance


        df['distance'] = df.apply(findDistance, axis=1)
        df = df.sort_values(by=["distance"])

        candidate = df.iloc[0]
        employee_name = candidate['ID']
        best_distance = candidate['distance']

        print(candidate[['ID', 'distance']].values)
    rimg = app.draw_on(frame, faces)
    result.write(rimg)
    # Display the resulting frame
    cv2.imshow('frame', rimg)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
result.release()
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()

#
# import  pickle
# file = open('database.pkl','rb')
# object_file = pickle.load(file)
# print(object_file)
# from find_distance import *
# distance = findEuclideanDistance([1,2], [4,6])
# print(distance)