import cv2
import numpy as np
import time
from threading import Thread
import signal


nulldisplay = np.zeros((720,1280,3), np.uint8)
cv2.putText(nulldisplay, 'Waiting for Camera', (450,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 1, cv2.LINE_AA)


def timeout_handler(num, stack):
    print("Received SIGALRM")
    raise Exception("FUBAR")

signal.signal(signal.SIGALRM, timeout_handler)

while True:
    # Create a VideoCapture object and read from input file
    # If the input is the camera, pass 0 instead of the video file name

    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)

    
    # Check if camera opened successfully
    if (cap.isOpened()== True): 
        
        while True:
            signal.alarm(2)
            try:
                ret, frame = cap.read()
            except Exception as ex:
                if "FUBAR" in str(ex):
                    print("Gotcha!")
                    ret = False
                else:
                    pass
                
            # Display the resulting frame
            if ret == True: 
                cv2.imshow('Frame', frame)
                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            else:
                break

        # When everything done, release the video capture object
        signal.alarm(0)
        cap.release()

    if (cap.isOpened()== False): 
        cap.release()
        frame = nulldisplay 
        cv2.imshow('Frame', frame)
        if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        
cv2.destroyAllWindows()