import numpy as np
import cv2, time

import base64
import redis

redisClient = redis.StrictRedis(host='localhost',
                                port=6379,
                                db=0)
ii = 0
last_i = -1

VIDEO_RECORD_DURATION_IN_SEC = 120
VIDEO_WRITE_FRAME_RATE = 25.0
ENABLE_DISPLAY = 1
IS_SOURCE_CAMERA = 1 # 0:Share memory, 1: Camera
#CAMERA_ID = 4
CAMERA_ID = 0

INPUT_WD = 1280
INPUT_HT= 720

RECORD_SPEED_ENABLE = 0




#print cap.get(3), cap.get(4), cap.get(7), cap.get(5)

def img_read_front(bimg=None):

    """
        -> Image read from Redis shared memory
        
    """
    
    global last_i
    img_data = redisClient.get('simage')     # Front_simage
    ii = int(redisClient.get('simage_cnt'))    #Front_simage_cnt 
    #print("ii",ii,last_i)
    if ii == last_i:# and False:        
        # time.sleep(0.05)
        #IMG_WIDTH = 1280
        #IMG_HEIGHT = 720
        #bimg = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype='uint8')        
        return False, bimg, ii
    else:
        last_i = ii
        jpg_original = base64.b64decode(img_data)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        image_buffer = cv2.imdecode(jpg_as_np, cv2.IMREAD_UNCHANGED)
        #img = cv2.cvtColor(image_buffer, cv2.COLOR_BGR2RGB)
        return True, image_buffer, ii


def main(): 

        dummy_frm_cnt = 0
        if(IS_SOURCE_CAMERA == 1):
            cap = cv2.VideoCapture(CAMERA_ID);
            cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,INPUT_WD);
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT,INPUT_HT);
            if not cap.isOpened():
                print("ERROR : Failed to open camera...",CAMERA_ID)
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        Name = 'vo_'+str(time.time())
        out = cv2.VideoWriter(Name+'.avi',fourcc, VIDEO_WRITE_FRAME_RATE, (INPUT_WD,INPUT_HT))
        file_output_vel = open(Name+".txt", "w")
        index = 0
        stFrmId = -1
        FrameId = 0

        while(1):
            #read frame from shared memory

            OBD_MSG = {"speed": 0,
                    "rpm":0,
                    "ts":0,
                    "is_valid": False}

            if(IS_SOURCE_CAMERA == 1):
                ret, frame = cap.read()   #read from camera
                index = index+1
            else:
                ret, frame, index = img_read_front() #read from shared memory

            if ret==True:
               

                capHt, capWd, _ = frame.shape
                if(capHt != INPUT_HT or capWd != INPUT_WD):
                  print("Mismatch in resolutions..", INPUT_WD,INPUT_HT,capWd, capHt)
                  exit(-1)

                print("writing index : ",index)
                dummy_frm_cnt = 0
                if stFrmId == -1:
                   stFrmId = index

                out.write(frame)
                file_output_vel.write('%d,%d,%d\n' % (FrameId+1,OBD_MSG["speed"],int(time.time()*1000)))
                FrameId = FrameId+1
                if(abs(index-stFrmId) >= VIDEO_WRITE_FRAME_RATE*VIDEO_RECORD_DURATION_IN_SEC):                    
                    #close current video and open a new video
                    out.release()
                    file_output_vel.close()

                    Name = 'vo_'+str(time.time())
                    out = cv2.VideoWriter(Name+'.avi',fourcc, VIDEO_WRITE_FRAME_RATE, (INPUT_WD,INPUT_HT))


                    file_output_vel = open(Name+".txt", "w")
                    FrameId = 0
                    stFrmId = -1
                if ENABLE_DISPLAY == 1:
                    frame = cv2.resize(frame,(320,180))
                    cv2.circle(frame,(300,20),10,(0,0,200),-1)
                    cv2.imshow('VideoRecording',frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:                
                dummy_frm_cnt += 1
                time.sleep(0.003)
                if (dummy_frm_cnt > 100):
                        break

        # Release everything if job is finished
        if(IS_SOURCE_CAMERA == 1):
                cap.release()
        out.release()
        file_output_vel.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

