# import the opencv library
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge, CvBridgeError

import os

camera_number = 0
PUB_ROS = False
CAPTURE_SPEED = 1 # in seconds

class image_converter:

  def __init__(self):
    if PUB_ROS:
        self.image_pub = rospy.Publisher("video_"+str(camera_number),Image, queue_size=10)
        self.bridge = CvBridge()
        
    self.name_pub = rospy.Publisher('video_'+str(camera_number)+'_name', Header, queue_size=10)
    

  def pub_image(self):
    global camera_number
    global CAPTURE_SPEED
    # define a video capture object
    cap = cv2.VideoCapture(camera_number)
    cap.set(3, 1280)
    cap.set(4, 720)
    
    last_ts = 0
    seq = 0
    while(True):
        # Capture the video frame
        # by frame
        ret, img = cap.read()
        ts = int(time.time()*1000)
        if ret and ((ts - last_ts) > (CAPTURE_SPEED*1000)):
            print((ts - last_ts))
            last_ts = ts
            header = Header(stamp=rospy.Time.now())
            seq = seq + 1
            if PUB_ROS:
                # resize image    
                image_temp=Image()
                image_temp = self.bridge.cv2_to_imgmsg(img, "bgr8")
                image_temp.header=header
            
            
            save_dir = "video"+str(camera_number)+"/"+str(header.stamp.secs)[:-2]+"/"
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            filename = save_dir+str(header.stamp.secs)+"_"+str(header.stamp.nsecs)+".jpg"

            header.frame_id = filename
            header.seq = seq
            print(seq)
            try:
                if PUB_ROS:
                    self.image_pub.publish(image_temp)
                self.name_pub.publish(header)
            except CvBridgeError as e:
                print(e)
              
            
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
    global image_topic_name
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
    rospy.init_node('video'+str(camera_number)+'_image', anonymous=False)
    try:
        ic.pub_image()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
