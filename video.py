import cv2
from threading import Thread
import time
import numpy as np

# defining a helper class for implementing multi-threading 
class WebcamStream :
    # initialization method 
    def __init__(self, stream_id=20):
        self.stream_id = stream_id # default is 0 for main camera 
        
        # opening video capture stream 
        self.vcap      = cv2.VideoCapture(self.stream_id)
        if self.vcap.isOpened() is False :
            print("[Exiting]: Error accessing webcam stream.")
            exit(0)
        fps_input_stream = int(self.vcap.get(5)) # hardware fps
        print("FPS of input stream: {}".format(fps_input_stream))
            
        # reading a single frame from vcap stream for initializing 
        self.grabbed , self.frame = self.vcap.read()
        if self.grabbed is False :
            print('[Exiting] No more frames to read')
            exit(0)
  
        self.stopped = True
       
        
        
        
    # method to start thread 
    def start(self):
        self.stopped = False
        self.t = Thread(target=self.update, args=())
        # self.t.daemon = True # daemon threads run in background 
        self.t.start()

    # method passed to thread to read next available frame  
    def update(self):
        while True :
            self.vcap = cv2.VideoCapture(self.stream_id)
            if self.stopped is True :
                break
            self.grabbed , self.frame = self.vcap.read()
            if self.grabbed is False :
                print('[Exiting] No more frames to read')
                self.stopped = True
                break 
        self.vcap.release()
    # method to return latest read frame 
    def read(self):
        return self.frame, self.grabbed
    # method to stop reading frames 
    def stop(self):
        self.stopped = True

# initializing and starting multi-threaded webcam input stream 
webcam_stream = WebcamStream(stream_id=10) # 0 id for main camera
webcam_stream.start()
print("start1")


nulldisplay = np.zeros((720,1280,3), np.uint8)
cv2.putText(nulldisplay, 'Waiting for Camera', (450,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 1, cv2.LINE_AA)

# processing frames in input stream
num_frames_processed = 0 
start = time.time()
print("start1")
while True :
    # if webcam_stream.stopped is True :
    #     break
    # else :
    print("start")
    frame, ret = webcam_stream.read()
    print("here")
    if ret == False:
        frame = nulldisplay
    # adding a delay for simulating video processing time 
    delay = 0.03 # delay value in seconds
    time.sleep(delay) 
    num_frames_processed += 1
    # displaying frame 
    cv2.imshow('frame' , frame)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
end = time.time()
webcam_stream.stop() # stop the webcam stream