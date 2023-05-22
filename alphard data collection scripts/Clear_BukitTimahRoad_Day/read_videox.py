
import sys
import cv2
import time
import os
import datetime


camera_number = 0
CAPTURE_SPEED = 1 # in seconds

class image_converter:

  def __init__(self):
     pass

  def pub_image(self):
    global camera_number
    global CAPTURE_SPEED
    # define a video capture object
    cap = cv2.VideoCapture(camera_number)
    cap.set(3, 1280)
    cap.set(4, 720)
    last_ts = 0
    seq = 0
    foldername = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
    save_dir = "video"+str(camera_number)+"/"+ foldername +"/"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    while(True):
        # Capture the video frame
        # by frame
        ret, img = cap.read()

        ts = int(time.time()*1000)
        if ret and ((ts - last_ts) > (CAPTURE_SPEED*1000)):
            print((ts - last_ts))

            last_ts = ts
            seq = seq + 1
           
            filename = save_dir + str(camera_number) + "_" + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f") +".jpg"

            print(filename)
            cv2.imwrite(filename, img)
            scale_percent = 40 # percent of original size
            width = int(img.shape[1] * scale_percent / 100)
            height = int(img.shape[0] * scale_percent / 100)
            dim = (width, height)
            
            frame_show = cv2.resize(img, dim)
            # Display the resulting frame
            cv2.imshow('video'+str(camera_number), frame_show)

        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    else:
        print(ret, "Frame Drop", (ts - last_ts))
    # After the loop release the cap object
    cap.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

def main(args):
    global camera_number
    global CAPTURE_SPEED
    camera_number = int(args[1])
    if (len(args) > 2):
        try:
            CAPTURE_SPEED = float(args[2])
            print("capture_speed in sec : ", CAPTURE_SPEED)
        except:
            CAPTURE_SPEED = 1

    print( "Camera id :",  camera_number)

    os.system("v4l2-ctl -d /dev/video{} --set-ctrl=hdr=1".format(camera_number))
    time.sleep(1)
    os.system("v4l2-ctl -d /dev/video{} --set-ctrl=tilt_absolute=-72000".format(camera_number))
    time.sleep(1)
    os.system("v4l2-ctl -d /dev/video{} -c frame_sync_mode=0".format(camera_number))

    ic = image_converter()

    try:
        ic.pub_image()

    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
