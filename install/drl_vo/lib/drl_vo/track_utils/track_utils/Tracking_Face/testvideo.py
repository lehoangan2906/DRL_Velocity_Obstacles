# Python program to save a 
# video using OpenCV
  
   
import cv2
import re
import threading
import logging
from time import sleep,time


logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger()

def run(ip,cam):  
    # Create an object to read 
    # from camera
    # source = 'rtsp://admin:123456a%40@172.21.111.'+str(ip)
    # if re.search(r"GStreamer:\s*NO", cv2.getBuildInformation()):
    #     logger.info("cv2 is built without gstreamer")
    #     video = cv2.VideoCapture(source)
    # else:
    #     pipeline = (
    #         "rtspsrc location={} latency=100 ! queue ! rtph265depay"
    #         " ! h265parse ! avdec_h265 ! videoscale ! video/x-raw,width=1920,height=1080"
    #         " ! videoconvert ! video/x-raw,format=BGR"
    #         " ! appsink emit-signals=true sync=false async=false drop=true".format(source)
    #     )

    #     logger.info("playing pipeline: %s", pipeline)
        # video = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    video = cv2.VideoCapture('rtsp://admin:123456a%40@172.21.111.'+str(ip))#111')
    
    # We need to check if camera
    # is opened previously or not
    if (video.isOpened() == False): 
        print("Error reading video file")
    
    # We need to set resolutions.
    # so, convert them from float to integer.
    frame_width = int(video.get(3))
    frame_height = int(video.get(4))
    
    size = (frame_width, frame_height)
    
    # Below VideoWriter object will create
    # a frame of above defined The output 
    # is stored in 'filename.avi' file.
    result = cv2.VideoWriter(str(cam)+'filename.mp4', 
                            cv2.VideoWriter_fourcc(*'mp4v'),
                            5, size)
 
    while (True):
        ret, frame = video.read()
        print(ret)
        if ret == True: 
    
            # Write the frame into the
            # file 'filename.avi'
            result.write(frame)
    
            # Display the frame
            # saved in the file
            # cv2.imshow(str(cam)+'Frame', cv2.resize(frame,(960,540)))
    
            # Press S on keyboard 
            # to stop the process
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
        # Break the loop

    
    # When everything done, release 
    # the video capture and video 
    # write objects
    video.release()
    result.release()
        
    # Closes all the frames
    cv2.destroyAllWindows()
   
print("The video was successfully saved")


if __name__ == "__main__":
    t1 = threading.Thread(target=run, args=([105,1]))
    
    t2 = threading.Thread(target=run, args=([108,2]))
    
    t3 = threading.Thread(target=run, args=([109,3]))

    # t4 = threading.Thread(target=run, args=([110,4]))
    t1.start()
    t2.start()
    t3.start()
    # t4.start()    
    # t1.join()
    # t2.join()
    # t3.join()
