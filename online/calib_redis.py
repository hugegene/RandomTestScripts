import os
import cv2
import numpy as np
from math import sqrt, atan, atan2, degrees, sin, cos, radians
import json
from statistics import mean
from util import *
import json
from calib_init import *
from datetime import datetime, time #ZE
import time as tm
import argparse
from cam2world import * #ZE
import csv #ZE
import shutil #ZE

""" REDIS """
import base64
import redis


ii = 0
last_i = -1


ENABLE_DISPLAY = 1
IS_SOURCE_CAMERA = 0 # 0:Share memory, 1: Camera

redisClient = redis.StrictRedis(host='localhost',
                                port=6379,
                                db=0)

def img_read():
    global last_i
    img_data = redisClient.get('simage3') #ZE
    
    jpg_original = base64.b64decode(img_data)
    jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8);
    image_buffer = cv2.imdecode(jpg_as_np, cv2.IMREAD_UNCHANGED)
        
    return True, image_buffer
""" REDIS """

def get_ego_speed():
    ego_speed = -1
    
    try:
        obd_msg = redisClient.get('obd_val')
        
        if len(obd_msg) <= 0:
            return -1

        obd_msg = obd_msg.decode('UTF-8')
        obd_msg = json.loads(obd_msg)
        ego_speed = float(obd_msg['speed'])
        return ego_speed
        
    except Exception as E:
        print(E)
        print('Failed to get_ego_speed()')
        return -1
    
parser = argparse.ArgumentParser()
#parser.add_argument("--file",type=str, required=True)
#args = parser.parse_args()

with open(os.path.join(sys.path[0], 'config/logconfig.yml'), 'r') as ymlfile:
  config = yaml.load(ymlfile, yaml.SafeLoader)

img_file = "redis"
outfile = "output/"+img_file + "_pt"
#print( img_file )
if not os.path.isdir("output"):
    os.mkdir("output")
if not os.path.isdir(outfile):
    os.mkdir(outfile)
if not os.path.isdir("heatmap"):
    os.mkdir("heatmap")
if not os.path.isdir("heatmap_seg"):
    os.mkdir("heatmap_seg")

skip = 1

img_ls = []

bypass = False
loggedfulldisk = False
imagereadfailurecount = 0
minspace = config['MIN_SPACE']*1000000000
#disk = config['LOG_PATH']
base = "/media/moocam/"
disk = ""
for folder in os.listdir(base):
    if os.path.isdir(os.path.join(base,folder)):
        for subdir in os.listdir(os.path.join(base, folder)): 
            #print(subdir)
            if subdir == "do_not_delete":
                print("found")
                disk = os.path.join(base, folder) + '/AutoCalibrationLogs/' 
print(disk)
#disk = config['LOG_PATH'] #REMOVE FOR REAL TEST
maindisk = disk
if not os.path.isdir(maindisk) and config['ENABLE_LOGGING'] == 1: #ZE
    os.mkdir(maindisk)

if os.path.isfile(maindisk + 'csv_log.csv') == False and config['ENABLE_LOGGING'] == 1:
    with open(maindisk + 'csv_log.csv', 'w') as c:
        writer = csv.writer(c)
        writer.writerow(['Timestamp', 'Pitch', 'Yaw', 'Frames Processed/Total Frames', 'Time Taken/min'])

while True:
    
    #data = yaml.load(open( "config/int_calib_{}.yaml".format(img_file) ), Loader=yaml.SafeLoader)
    data = cv2.FileStorage("config/int_calib_{}.yaml".format(img_file), cv2.FILE_STORAGE_READ) #ZE
    calibrate = onlineCalibration( data )
    disk = maindisk + str(calibrate.timestamp) + "/"
    if not os.path.isdir(maindisk + str(calibrate.timestamp) + "/Frames/") and config['ENABLE_LOGGING'] == 1 and config['LOG_FRAMES'] == 1: #ZE
        os.mkdir(maindisk + str(calibrate.timestamp) + "/Frames/")
    if config['ENABLE_LOGGING'] == 1:
        with open(disk + 'logs.txt', 'w') as t:
            t.write(img_file)

    prev_cnt = 0

    st_time = -1

    while True:
        loopstarttime = tm.time()
        #ret, image1, index = img_read()
        if config['ENABLE_SPEED_LIMITATION'] == 1:
            slavestarttime = tm.time()
            currentvehiclespeed = get_ego_speed()
            slavetime = tm.time() - slavestarttime
            if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                with open(disk + 'logs.txt', 'a') as t:
                    t.write('\nTime to get speed in ms: {}'.format(slavetime*1000))
            else:
                print('Time to get speed in ms {}'.format(slavetime*1000))
        else:
            currentvehiclespeed = -1

        if config['ENABLE_LOGGING'] == 1:
            calibrate.remainingstoragespace = shutil.disk_usage(disk)[2]
        if config['ENABLE_SPEED_LIMITATION'] == 1 and currentvehiclespeed > config['MAX_SPEED'] and currentvehiclespeed >= 0:
            s = datetime.now()
            if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                with open(disk + 'logs.txt', 'a') as t:
                    t.write('\nVehicle moving faster than maximum speed. Current speed: ' + str(currentvehiclespeed) + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))
            else:
                print('Vehicle moving faster than maximum speed. Current speed: ' + str(currentvehiclespeed) + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))
            calibrate.skipped_frames_count += 1
            tm.sleep(1)
            continue
        
        elif config['ENABLE_SPEED_LIMITATION'] == 1 and currentvehiclespeed < config['MIN_SPEED'] and currentvehiclespeed >= 0:
            s = datetime.now()
            if config['ENABLE_LOGGING'] == 1  and calibrate.remainingstoragespace > minspace:
                with open(disk + 'logs.txt', 'a') as t:
                    t.write('\nVehicle moving slower than minimum speed. Current speed: ' + str(currentvehiclespeed) + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))
            else:
                print('Vehicle moving slower than minimum speed. Current speed: ' + str(currentvehiclespeed) + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))
            calibrate.skipped_frames_count += 1
            tm.sleep(1)
            continue

        elif config['ENABLE_SPEED_LIMITATION'] == 1 and currentvehiclespeed < 0:
            s = datetime.now()
            if config['ENABLE_LOGGING'] == 1  and calibrate.remainingstoragespace > minspace:
                with open(disk + 'logs.txt', 'a') as t:
                    t.write('\nFailed to get ego speed. Speed filtering inactive.' + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))
            else:
                print('Failed to get ego speed. Speed filtering inactive.' + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))

        if config['ENABLE_TIME_LIMITATION'] == 1 and (datetime.now().time() >= time(int(config['LATEST_TIME'].split(',')[0]), int(config['LATEST_TIME'].split(',')[1])) or datetime.now().time() <= time(int(config['EARLIEST_TIME'].split(',')[0]), int(config['EARLIEST_TIME'].split(',')[1]))):
            if config['ENABLE_LOGGING'] == 1  and calibrate.remainingstoragespace > minspace:
                with open(disk + 'logs.txt', 'a') as t:
                    t.write('\nCurrent time is outside desired timing: ' + str(datetime.now().time()) + ' Current speed: ' + str(currentvehiclespeed))
            else:
                print('Current time is outside desired timing: ' + str(datetime.now().time()) + ' Current speed: ' + str(currentvehiclespeed))
            calibrate.skipped_frames_count += 1
            tm.sleep(1)
            continue
      
        ret, image1 = img_read() #ZE
        if calibrate.total_img_cnt == 0:
            sameimage = False
        else:
            sameimage = (oldimage==image1).all() #ZE
 
        oldimage = image1 #ZE

        if not sameimage: #ZE #if ret == True:
            
            if config['LOG_FRAMES'] == 1 and config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                d = datetime.now()
                cv2.imwrite(disk + "/Frames/" + str("{}-{}-{}_{}.{}.{}.{}".format(d.year, d.month, d.day, d.hour, d.minute, d.second, d.microsecond)) + '.jpg', image1)

            imagereadfailurecount = 0 #ZE

            if calibrate.FailedCalibration == True:
                print("failedcalib true")
                if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                    with open(maindisk + 'csv_log.csv', 'a') as c:
                        writer = csv.writer(c)
                        writer.writerow([calibrate.timestamp, 'NIL', 'NIL', 'Calibration Failed', str((tm.time() - calibrate.starttime)/60)])
                else:
                    print("""Failed calibration, please perform the calibration again at places :
                        1. without Traffic lights
                        2. on a straight road
                        3. On roads with good lane lines
                        """)
                    print(str([calibrate.timestamp, 'NIL', 'NIL', 'Calibration Failed', str((tm.time() - calibrate.starttime)/60)]))
                break
            
            # image must be set to w=640, h=360
            imgfx = calibrate.inp_w / image1.shape[1]
            imgfy = calibrate.inp_h / image1.shape[0]

            image1 = cv2.resize(image1, (0,0), fx = imgfx, fy = imgfy )

            if not calibrate.calib_finished and not bypass:

                st_time = tm.time()

                img_ls.append(image1)

                if len( img_ls ) == skip +1 :
                    #print( img_ls[0].shape, img_ls[-1].shape )
                    calibrate.calib( img_ls[0], img_ls[-1] )

                    # print FPS
                    if st_time != -1: 
                        timetaken = tm.time() - st_time
                        fps = 1/ ( timetaken )
                        if fps < 1000: # raise an issue
                            timetaken = int( timetaken*100000 )/100
                            kp_perc = int( calibrate.kp_t*100000 )/100
                            fl_perc = int( calibrate.fl_t*100000 )/100
                            pr_perc = int( calibrate.pr_t*100000 )/100
                            int_perc = int( calibrate.int_t*100000 )/100
                            s = datetime.now()
                            if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                                text1 = "\nIdx  " + str(calibrate.total_img_cnt) + "  time in ms:  " + str(timetaken) + "  kp_perc:  " + str(kp_perc) + "   fl_perc:  " + str(fl_perc) + "  pr_perc:  " + str(pr_perc) + "  int_perc:  " + str(int_perc) 
                                text2 = '\nCurrent Vehicle Speed: ' + str(currentvehiclespeed) + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second))
                                with open(disk + 'logs.txt', 'a') as t:
                                    t.write(text1)
                                    t.write(text2)
                            else:
                                print( "Idx ", calibrate.total_img_cnt,  " time in ms: ", timetaken, " kp_perc: ", kp_perc, "  fl_perc: ", fl_perc, " pr_perc: ", pr_perc, " int_perc: ", int_perc )
                                print('Current Vehicle Speed: ' + str(currentvehiclespeed) + ' Current Datetime: ' + str("{}-{}-{}_{}.{}.{}".format(s.year, s.month, s.day, s.hour, s.minute, s.second)))
                    if False:#enable_draw
                        draw_img = np.copy( img_ls[-1] )
                        draw_img = cv2.resize(draw_img, (0,0), fx = 2, fy = 2 )
                        line_img, pitch_img = draw( draw_img, calibrate, prev_cnt )
                        out_img = cv2.hconcat( (line_img, pitch_img) )
                        out_img = cv2.resize(out_img, (0,0), fx = 1/2, fy = 1/2 ) 
                        cv2.imwrite("{}/{}_pitch.jpg".format(outfile, cnt), out_img)

                    img_ls = img_ls[1:]
            else:
                #ZE------------------------------------------------------------------------------------------------------
                if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                    maxLngDistGridmts = 20
                    c2w = Cam2WldMap(disk + 'mooCam_calib_out.yml').loadCam2WldMaps()
                    lngGridIntervalmts = 2
                    latGridIntervalmts = 2
                    horizonLngDistmts = 10000
                    frame = image1
                    frame = cv2.resize(frame, (1280, 720))
                    lat = range(-8,8+1,latGridIntervalmts)
                    lng = range(2,maxLngDistGridmts+1,lngGridIntervalmts)
                    DrawGrid = frame.copy()
                    #draw horizontal lines
                    for ln in lng:
                       WldPtRev0 = (-10,ln)
                       CamPt0 = c2w.mapWorldPtToCamPt(WldPtRev0)
                       WldPtRev1 = (10,ln)
                       CamPt1 = c2w.mapWorldPtToCamPt(WldPtRev1)
                       print(WldPtRev0, CamPt0, WldPtRev1, CamPt1)
                       cv2.line(DrawGrid, (int(CamPt0[0]),int(CamPt0[1])), (int(CamPt1[0]),int(CamPt1[1])), green, 1)
                    #draw horizon line at long=100mt 
                    WldPtRev0 = (-20000,horizonLngDistmts)
    
                    CamPt0 = c2w.mapWorldPtToCamPt(WldPtRev0)
                    WldPtRev1 = (20000,horizonLngDistmts)
                    CamPt1 = c2w.mapWorldPtToCamPt(WldPtRev1)
    
                    print("drawing horizon line")
                    print(CamPt0)
                    print(CamPt1)
    
                    cv2.line(DrawGrid, (int(CamPt0[0]),int(CamPt0[1])), (int(CamPt1[0]),int(CamPt1[1])), (0, 0, 255), 1)
    
        	    # TODO : Draw vertical lines upto maxLngDistGridmts
                    for la in lat:
                        WldPtRev0 = (la, 2)
                        CamPt0 = c2w.mapWorldPtToCamPt(WldPtRev0)
                        WldPtRev1 = (la, horizonLngDistmts)
                        CamPt1 = c2w.mapWorldPtToCamPt(WldPtRev1)
                        if la == 0:
                            color = (255, 0, 0)
                        else:
                            color = (0, 255, 0)
                        cv2.line(DrawGrid, (int(CamPt0[0]),int(CamPt0[1])), (int(CamPt1[0]),int(CamPt1[1])), color, 1)
                    cv2.imwrite(disk + 'gridimage.jpg', DrawGrid)
                    #--------------------------------------------------------------------------------------------------------

                break
                """# undistortion
                image1 = cv2.remap(image1, calibrate.mapx, calibrate.mapy, cv2.INTER_LINEAR)
                """


                #
                #
                #
                #
                #
                #
                #
    #ZE--------------------------------------------
        else:
            calibrate.skipped_frames_count += 1
            if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                with open(disk + 'logs.txt', 'a') as t:
                    t.write('\nEntered same image')
            else:
                print('Entered same image') 
            if imagereadfailurecount >= 3:
                imagereadfailurecount = 0
                if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
                    with open(disk + 'logs.txt', 'a') as t:
                        t.write('\nImage Read Failure')
                else:
                    print('Image Read Failure')
            else:
                imagereadfailurecount += 1
            tm.sleep(1)
    #----------------------------------------------
                
        if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace < minspace:
            if loggedfulldisk == False:
                with open(disk + 'logs.txt','a') as t:
                    t.write('\nMinimum disk space reached. Logging stopped.')
                loggedfulldisk = True
            print('Minimum disk space reached. Logging stopped.')
        else:
            loggedfulldisk = False

        looptime = tm.time() - loopstarttime
        if config['ENABLE_LOGGING'] == 1 and calibrate.remainingstoragespace > minspace:
            with open(disk + 'logs.txt', 'a') as t:
                t.write('\nLoop time: {}'.format(looptime*1000))
                t.close()
        else:
            print('Loop time: {}'.format(looptime*1000))

    if config['ENABLE_LOOP'] != 1:
        break
