import sys
import os
import cv2
import numpy as np
from math import sqrt, atan, atan2, degrees, sin, cos, radians, tan
import json
from statistics import mean
from util import *
import json
import yaml
import time
import datetime
import csv #ZE
import shutil #ZE
# the lines and vanishing point are for the next frame, not the first image

eps = 0.000000000001
"""
Steps:
1. remove flow lines that are short, usually inaccurate
2. Merge multiple lines together.
3. remove flow lines from image edges since they are usually inaccurate.
4. 
"""
roll_angle = 0
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 4,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

with open(os.path.join(sys.path[0], 'config/logconfig.yml'), 'r') as ymlfile:
            config = yaml.load(ymlfile, yaml.SafeLoader)

minspace = config['MIN_SPACE']*1000000000
#disk = config['LOG_PATH']
base = "/media/moocam/"
disk = ""
for folder in os.listdir(base):
    if os.path.isdir(os.path.join(base, folder)):
        for subdir in os.listdir(os.path.join(base, folder)): 
            #print(subdir)
            if subdir == "do_not_delete":
                print("found")
                disk = os.path.join(base, folder) + '/AutoCalibrationLogs/' 

#disk = config['LOG_PATH']#REMOVE FOR REAL TEST
maindisk = disk

class onlineCalibration:
    def __init__( self, data ):
        global disk
        x = datetime.datetime.now() #ZE
        self.starttime = time.time() #ZE
        self.remainingstoragespace = shutil.disk_usage(disk)[2]
        self.timestamp = ("{}-{}-{}_{}.{}.{}".format(x.year, x.month, x.day, x.hour, x.minute, x.second)) #ZE
        if not os.path.isdir(maindisk + str(self.timestamp) + "/") and config['ENABLE_LOGGING'] == 1: #ZE
            os.mkdir(maindisk + str(self.timestamp) + "/")
        self.lk_params = lk_params
        #self.cam_calib = np.array( data["int_calib"] )
        self.intrinsic_timestamp = data.getNode('ts').string() #ZE
        self.cam_calib = data.getNode('int_calib').mat() #ZE
        print( self.cam_calib )
        #self.distor = np.array( data["distor"] )
        self.distor = data.getNode('distor').mat() #ZE
        #self.cam_h = data["cam_h"]/100 # convert from cm to meters
        self.cam_h = (float)(data.getNode('cam_h').real())/100 #ZE
        print(self.cam_h)
        #self.lat_range = data["lat_range"]
        self.lat_range = (int)(data.getNode('lat_range').real()) #ZE
        #self.long_range = data["long_range"]
        self.long_range = (int)(data.getNode('long_range').real()) #ZE
        #self.res_per_m = data["res_per_m"]
        self.res_per_m = (int)(data.getNode('res_per_m').real()) #ZE
        #self.cam2bumper_dist = data["cam2bumper_dist"]/100 # convert from cm to meters
        self.cam2bumper_dist = (float)(data.getNode('cam2bumper_dist').real())/100 #ZE
        
        #self.mfromleft = data["CmFromLeft"]/100 # convert from cm to meters
        self.mfromleft = (float)(data.getNode('CmFromLeft').real())/100 #ZE
        #self.mfromright = data["CmFromRight"]/100 # convert from cm to meters
        self.mfromright = (float)(data.getNode('CmFromRight').real())/100 #ZE
        self.lat_off = self.mfromleft - ( self.mfromleft + self.mfromright )/2
        #self.VehicleName = data["VehicleName"]
        self.VehicleName = data.getNode('VehicleName').string() #ZE
        #print( self.lat_off )
        
        #self.img_w = data["img_w"]; self.img_h = data["img_h"]
        self.img_w = (int)(data.getNode('img_w').real()); self.img_h = (int)(data.getNode('img_h').real()) #ZE
        self.inp_w = self.img_w/2
        self.inp_h = self.img_h/2
        self.cen_x = int( self.img_w/2 )
        self.cen_y = int( self.img_h/2 )
        
        self.x_list = [0,self.img_w]
        
        # avoid points near the edge of the image
        self.bound_xmin = self.img_w* 1/10
        self.bound_xmax = self.img_w* 9/10
        self.bound_ymin = self.img_h* 1/5
        self.bound_ymax = self.img_h* 4/5
        
        # image down sizing
        self.downsize = 2
        #print( self.downsize )
        
        self.vp_avg = None
        self.y_avg = -1000
        self.x_avg = -1000
        self.vp_ls = []
        self.vp_calibrated = None # used for limiting the distance estimation.
        
        self.pitch_norm_ls = [] # constrained pitch ls
        self.last_pitch_segment_cnt = 0 # since constrained pitch ls might be different size than un constrained pitch segment, this is used.
        self.pitch_diff = [] # used to get the abs difference between pitch
        self.yaw_norm_ls = []
        self.yaw_ls = []
        self.pitch_ls = []
        self.norm_length = 1
        
        # test the a, b, c, d of flow lines
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.cnt = 0
        
        # filtering
        self.yaw_filt = 60
        self.pitch_filt = 40
        self.p_diff_lim = 0.25 # limit change in pitch

        # further limit the pitch to +- 1 degree from mean value
        self.mean_deg = 0
        self.mean_cnt = 0
        self.lim_from_mean_deg = 1.0
        self.deg_abs_lim = 10
        self.wrong_vp_cnt = 0
        
        
        # line filtering regions
        self.x_region_min = 2/5 * self.img_w / sqrt(self.cam_calib[0,0]/600)
        self.x_region_max = self.img_w - self.x_region_min
        self.y_region_min = 2/5* self.img_h / sqrt(self.cam_calib[0,0]/600)
        self.y_region_max = self.img_h - self.y_region_min
        #print( self.x_region_min, self.x_region_max )
        #print( self.y_region_min, self.y_region_max )
        
        """ determine number of pixel needed for 1 degree of yaw and pitch"""
        # get pixel corresponding to pitch.
        pitchPixrange = []
        for i in [int(self.y_region_min), int(self.y_region_max)]:
            p, y = get_py_from_vp( int( self.img_w/2 ), i, self.cam_calib )
            pitchPixrange.append( degrees(p) )
        self.pitchPerPix = ( pitchPixrange[1] - pitchPixrange[0] ) / ( int(self.y_region_max) - int(self.y_region_min) )
        #print( 1/ self.pitchPerPix)
        
        yawPixrange = []
        for i in [int(self.x_region_min), int(self.x_region_max)]:
            p, y = get_py_from_vp( i, int( self.img_h/2 ), self.cam_calib )
            yawPixrange.append( degrees(y) )
        self.yawPerPix = ( yawPixrange[1] - yawPixrange[0] ) / ( int(self.x_region_max) - int(self.x_region_min) )
        disk = maindisk + str(self.timestamp) + "/" #ZE
        if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
            with open(disk + 'logs.txt', 'a') as t:
                t.write('\n' + str(1/ self.yawPerPix))
                t.close()
        else:
            print( 1/ self.yawPerPix) #ZE
        
        self.hom_mat = None
        
        # limit the number of keypoints  for both sides on the top of the image:
        self.max_toplr_kp = 10
        self.max_botlr_kp = 50
        self.all_kp = []
        
        # filter images based on keypoints
        self.min_kp_total = 10 # reject any image with less than 10 keypoints
        self.min_intercnt = 0 # filter images with no intersections
        
        # for fusing previous points to current points
        self.prev_pts = []
        self.old_pt_cnt = 0 # the count for where the old points starts

        # limit intersections based on angles        
        self.angle_gap = radians(30)
        
        # For checking the segments used for pitch calculations
        self.img_cnt = 0 # for checking how long the calibration should last
        self.total_img_cnt = 0
        self.skipped_frames_count = 0
        self.segments = [] # store the data from all segments
        self.caibration_length = config['CALIBRATION_LENGTH'] # Check if calibration is finished every x frame; Also draws the heatmap each x frame. Cannot be less than pitch interval. 

        self.pitch_interval = 3 # Number of frames to collect intersection points for heatmap generation.
        self.maxframe = 9000 # maximum of x frames to get all the filtframe. 10 minutes at 900 frames per min
        self.filtframes = 1000 #1500 # number of filtframe needed
        
        # filtering params
        self.yaw_filt_lim = 2
        self.pitch_filt_lim = 1
        
        # multiple image voting angle
        #self.vp_y_votecnt = np.zeros( ( int( self.img_h/2 ), int( self.img_w/2 ) ) ) # for all the segments
        self.vp_seg_xpos = [0]*int(self.inp_w)
        self.vp_seg_ypos = [0]*int(self.inp_h)
        self.vp_seg_dict = dict()
        self.vp_seg_dict_ls = []
        self.vp_seg_angle_ls = []

        self.segment_cnt = 0
        self.vp_votecnt_ls = []
        self.votecnt_cnt = 0 # number of votecnt collected
        self.all_inters_pt = []
        
        # for heatmap angle gen
        self.angles = []
        self.pitch_heatmap_offset = 1
        self.yaw_heatmap_offset = 2
        self.yaw_window = int ( 1/ self.pitchPerPix )
        self.pitch_window = int( 1/ self.pitchPerPix )
        # print( self.pitch_window, self.yaw_window )
        self.minFrameCnt = 100 #3600 #IMPORTANT: determine the minimum time needed for calibration. Set at 4 minutes with 900 frames per min = 3600

        self.angle_ls = []
        self.img_cnt_ls = []
        
        self.calib_finished = False
        self.FailedCalibration = False

        self.kp_t = 0
        self.fl_t = 0
        self.pr_t = 0
        self.int_t =0
        self.min_angle = tan(radians(20))
        self.max_angle = tan(radians(85))
        self.img_trim_lim_y = int(self.inp_h* 1/4.5)
        self.img_trim_lim_x = int(self.inp_w* 1/9.5)
        #self.img_trim_lim_y = 0
        self.cen_h = self.inp_h/2 - self.img_trim_lim_y
        self.cen_w = self.inp_w/2
        
    # reset all the data for each function call.
    def reset(self, ):
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        
    
    # used to get the flow lines, performs merging, boundary check and removing horizontal / vertical line
    # also asign the lines according to the quadrants, and filters the points from top quadrants if there are enoguh points from bottom quadrants.
    def getFlowLines(self, image1, image2):
        # get optical flow
        #print( image1.shape , image2.shape )
        start_t = time.time()
        imgfx = int(self.img_w/self.inp_w)
        imgfy = int(self.img_h/self.inp_h)

        image1 = image1[ self.img_trim_lim_y : -self.img_trim_lim_y, self.img_trim_lim_x:-self.img_trim_lim_x, : ]
        image2 = image2[ self.img_trim_lim_y : -self.img_trim_lim_y, self.img_trim_lim_x:-self.img_trim_lim_x, : ]
        disk = maindisk + str(self.timestamp) + "/" #ZE
        if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
            text = '\n' + str(image1.shape) + ' ' + str(image2.shape)
            with open(disk + 'logs.txt', 'a') as t:
                t.write(text)
                t.close()
        else:
            print(image1.shape, image2.shape) #ZE
        prior_gray = cv2.cvtColor( image1, cv2.COLOR_BGR2GRAY) 
        post_gray = cv2.cvtColor( image2, cv2.COLOR_BGR2GRAY) 

        point_ls = cv2.goodFeaturesToTrack(prior_gray,600,0.001,int(20/self.downsize),10/self.downsize)
 
        #self.old_pt_cnt = len( point_ls )
        
        # append points from previous frames to current frame # does not work, reduces the number of filtered image by a lot.
        
        #point_ls = list(point_ls)
        #for p in self.prev_pts:
        #    point_ls.append( [ p ] )
        #self.prev_pts=[]

        point_ls = np.array(point_ls, dtype=np.float32)
        self.kp_t = time.time() - start_t
        start_t = time.time()
        # optical flow      
        p1, st, err = cv2.calcOpticalFlowPyrLK(prior_gray, post_gray, point_ls, None, **lk_params)
        self.fl_t = time.time() - start_t
        start_t2 = time.time()
        if p1 is not None: good_new = p1; good_old = point_ls
        
        # draw filtered and unfiltered line
        self.filt_line = []

        kp_quad = [0,0,0,0] # top right, top left, bottom right, bottom left
        filt_line = [ [],[],[],[] ]
        
        filt1= 0
        filt2 = 0
        cnt = 0
        last_cnt = 0
        for new,old in zip(good_new, good_old):
            cnt+=1
            if kp_quad[2] + kp_quad[3] > 80: break
            
            a = ( new[0][0] + self.img_trim_lim_x )*imgfx
            b = ( new[0][1] + self.img_trim_lim_y )*imgfy
            #if not self.featurepoint_check(a,b): continue
            
            cnt0 =0; cnt1=0
            if b > self.cen_y: cnt1=1
            if a > self.cen_x: cnt0=1
            
            quad = cnt1*2+cnt0
            if quad in [0,1]: 
                if kp_quad[ quad ] == self.max_toplr_kp: continue
            else:
                if kp_quad[ quad ] == self.max_botlr_kp: continue

            c = ( old[0][0] + self.img_trim_lim_x )*imgfx
            d = ( old[0][1] + self.img_trim_lim_y )*imgfy
            #if not self.featurepoint_check(c,d): continue
            
            # remove keypoints near the center point
            #if not self.centerpoint_check( a,b ): continue
            #if not self.centerpoint_check( c,d ): continue

            """ optical flow filtering """
            # filter out opt flow lines that are too short or too long
            #if not 10 <  sqrt( abs(a-c)**2 + abs(b-d)**2) < 200: # Fixed bug which affect pitch and yaw accuracy.
            #    continue

            """ angle filtering """
            m = (b-d)/( (a-c)+eps)
            
            if not self.min_angle < abs( m ) < self.max_angle: filt2+=1;continue
            
            # filter out flow lines that moves towards the center (remove flow from car moving away from car)
            if abs( a - self.img_w/2 ) > abs( c- self.img_w/2 ) or abs( b - self.img_h/2 ) > abs( d - self.img_h/2 ):
                filt1+=1
                continue

            bias = (d  - m*( c ))
            kp_quad[ quad ] += 1
            last_cnt = cnt
            filt_line[quad].append( [m, 1, bias] )
            

        self.pr_t = time.time() - start_t2
        disk = maindisk + str(self.timestamp) + "/" #ZE
        if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
            text1 = "\nfiltering by different methods:  " + str(filt1) + ' ' + str(filt2)
            text2 = "\nlast count with accepted kp :  " + str(len( good_new )) + ' ' +str(last_cnt)
            text = "\nIdx  " + str(self.total_img_cnt) + "   keypoints in top right, top left, bottom right, bottom left quadrants:  " + str(kp_quad)
            with open(disk + 'logs.txt', 'a') as t:
                t.write(text1)
                t.write(text2)
                t.write(text)
                t.close()
        else:
            print( "filtering by different methods: ", filt1, filt2 ) #ZE
            print( "last count with accepted kp : ",len( good_new ), last_cnt ) #ZE
            print( "Idx ", self.total_img_cnt, "  keypoints in top right, top left, bottom right, bottom left quadrants: ", kp_quad  ) #ZE
        return filt_line

    # ignore feature points near the edge of the image as the optical flow from those areas are usually wrong.
    def featurepoint_check(self, x, y):
         if not self.bound_xmin < x < self.bound_xmax: return False
         if not self.bound_ymin < y < self.bound_ymax: return False
         return True
    
    # ignore feature points near the center height pixel as these are usually wrong.
    def centerpoint_check(self, x, y):
         center_y = self.cen_y - self.mean_deg / self.pitchPerPix
         
         dist_x = abs( x - self.cen_x )
         dist_y = abs( y - self.cen_y )
         
         if abs( dist_y/( dist_x+ eps) ) < 0.2 : # control the area beside central hiwght, higher number means more filtering
             return False
         return True
    
    # Only accept optical flow lines that pass through a region.
    def line_check(self, x, y):
         if not self.x_region_min < x < self.x_region_max: return False
         if not self.y_region_min < y < self.y_region_max: return False
         return True
    
    # calibration process
    def calib( self, image1, image2 ):
        self.img_cnt += 1
        self.total_img_cnt += 1
        
        self.reset()
        
        if not self.calib_finished:
            
            # check if calibration is taking too long
            if self.total_img_cnt > self.maxframe:
                self.FailedCalibration = True
            
            # Use 2nd image as prior since the car moves forward, some point in image1 is not in image2.
            filt_line = self.getFlowLines(image2, image1)
                
            # line data format: m, cnt, b, quadrant

            # calculate untersection            
            st_int = time.time()
            self.calc_intersection(filt_line)
            disk = maindisk + str(self.timestamp) + "/" #ZE
            if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
                text = "\nIdx  " + str(self.total_img_cnt) + "  number of intersection points:  " + str(len(self.all_inters_pt))
                with open(disk + 'logs.txt', 'a') as t:
                    t.write(text)
                    t.close()
            else:
                print( "Idx ", self.total_img_cnt, " number of intersection points: ", len(self.all_inters_pt) ) #ZE
            
            self.int_t = time.time() - st_int
            # make sure each frames is treated the same regardless of the number of inter_pt
            if len( self.all_inters_pt ) > self.min_intercnt:
                votecnt = 1000/len( self.all_inters_pt )
                self.votecnt_cnt += 1 # number of times votecnt is collected
                self.img_cnt_ls.append( self.total_img_cnt )
                for x_pos, y_position in self.all_inters_pt :
                    
                    idx_ypos = int(y_position/2)
                    idx_xpos = int( x_pos/2 )
                    self.vp_seg_ypos[ idx_ypos ] += votecnt
                    self.vp_seg_xpos[ idx_xpos ] += votecnt
                    
                    idx = "{}_{}".format( int(y_position/2), int( x_pos/2 ) )
                    if idx in self.vp_seg_dict.keys():
                        self.vp_seg_dict[ idx ] += votecnt
                    else:
                        self.vp_seg_dict[ idx ] = votecnt

            
            """ Save the pitch information for testing also save the un constrained pitch ls to determine whether the road is bumpy or not """
            if self.votecnt_cnt % self.pitch_interval == 0 and self.votecnt_cnt > self.cnt:
                #info = self.pitch_norm_ls[-self.pitch_interval:]
                #yaw_info = self.yaw_norm_ls[-self.pitch_interval:]
                self.cnt += self.pitch_interval
                
                self.heatmap_angle_gen( self.vp_seg_xpos, self.vp_seg_ypos )
                
                #if np.max( self.vp_seg_votecnt ) > 0:
                #    images = ( ( self.vp_seg_votecnt/ (np.max( self.vp_seg_votecnt ) ) )*255 )
                #else: images=self.vp_seg_votecnt
                
                #images = np.clip( images, 0, 255 )
                #images = images.astype(np.uint8)
                #images = cv2.applyColorMap(images, cv2.COLORMAP_JET)
                #cv2.imwrite( "heatmap_seg/{}.jpg".format(self.total_img_cnt +10000000000 ), images )
                
                #print( "current segment: pitch:", -1*degrees( self.angles[0] ), "   yaw:" , degrees( self.angles[1] )  )
                self.vp_seg_dict_ls.append( self.vp_seg_dict )
                self.vp_seg_xpos = [0]*int(self.inp_w)
                self.vp_seg_ypos = [0]*int(self.inp_h)
                self.pitch_ls.append( -1*degrees( self.angles[0] ) )
                self.yaw_ls.append( degrees( self.angles[1] ) )
                disk = maindisk + str(self.timestamp) + "/" #ZE
                if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
                    text = "\nIdx  " + str(self.total_img_cnt) + "  pitch angle:  " + str(self.pitch_ls[-1]) + "  yaw angle:  " + str(self.yaw_ls[-1])
                    with open(disk + 'logs.txt', 'a') as t:
                        t.write(text)
                        t.close()
                else:
                    print( "Idx ", self.total_img_cnt, " pitch angle: ", self.pitch_ls[-1], " yaw angle: ", self.yaw_ls[-1] ) #ZE
                self.vp_seg_dict = dict()


                
                self.last_pitch_segment_cnt = self.total_img_cnt
                

                
                
            # Save calibration results, pitch and vp point.
            if self.img_cnt >= self.caibration_length:
                # accepted_mean_pitch, accepted_segment = self.filter_segments() # for testing purposes
                self.img_cnt = 0
                if self.heatmap_angle_check():
                    disk = maindisk + str(self.timestamp) + "/" #ZE
                    if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:

                        text = "\ntotal calibration time: {} minutes".format( self.total_img_cnt/900)

                        with open(disk + 'logs.txt', 'a') as t:
                            t.write(text)
                            t.write("\nfinished")
                            t.close()
                    else:
                        print( "total calibration time: {} minutes".format( self.total_img_cnt/900 ) ) #ZE
                        print("finished") #ZE     

                    self.gen_BEVhom( -1*float(self.angles[0]), float(self.angles[1])  )
                    self.vp_calibrated = self.vp_avg
                    
                    pts_dst = []
                    pts_src = []
                    # Camera based calibration, world system based on camera
                    for i in range(-5, 6, 5):
                        pts_dst.append( [i*100,( 2 )*100] )
                        pts_src.append( self.getPix( (i,2 - self.cam2bumper_dist ) ) )
                        
                        pts_dst.append( [i*100,(20  )*100] )
                        pts_src.append( self.getPix( (i,20 - self.cam2bumper_dist ) ) )
                    disk = maindisk + str(self.timestamp) + "/" #ZE
                    if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
                        with open(disk + 'logs.txt', 'a') as t:
                            t.write('\n' + str(pts_src) + ' ' + str(pts_dst))
                            t.close()
                    else:
                        print( pts_src, pts_dst ) #ZE

                    img2wld_h, _ = cv2.findHomography( np.array(pts_src, dtype=np.float32), np.array(pts_dst, dtype=np.float32) )
                    print( img2wld_h )
                    _, wld2img_h = cv2.invert(img2wld_h)
                    print( wld2img_h)

                    pitch = degrees(-1*float(self.angles[0])) #ZE
                    yaw = degrees(float(self.angles[1])) #ZE
                    imgcnt = self.total_img_cnt #ZE
                    if config['ENABLE_LOGGING'] == 1 and shutil.disk_usage(maindisk)[2] > minspace:
                        with open(maindisk + 'csv_log.csv', 'a') as c:
                            writer = csv.writer(c)
                            writer.writerow([self.timestamp, pitch, yaw, str(imgcnt) + '/' + str(imgcnt + self.skipped_frames_count), str((time.time() - self.starttime)/60)])
                            c.close()

                    #f = cv2.FileStorage("config/moobox_out.yml", cv2.FileStorage_WRITE)
                    #f = cv2.FileStorage("config/mooCam_calib_out.yml", cv2.FileStorage_WRITE) #ZE
                    disk = maindisk + str(self.timestamp) + "/" #ZE
                    if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace: 
                        f = cv2.FileStorage(disk + "mooCam_calib_out.yml", cv2.FileStorage_WRITE) #ZE
                    else:
                        f = cv2.FileStorage("config/mooCam_calib_out.yml", cv2.FileStorage_WRITE)
 
                    #f.write( 'VER', "1.0" )
                    f.write( 'VER', "2.0" ) #ZE
                    f.write( 'ALGO', "GND_HOMOGRAPHY" )
                    x = datetime.datetime.now()
                    timestamp = ("{}-{}-{}_{}.{}.{}".format(x.year, x.month, x.day, x.hour, x.minute, x.second))
                    f.write( 'ts', timestamp )
                    f.write( 'i_ts', self.intrinsic_timestamp) #ZE
                    f.write( 'CamH', int( self.cam_h*100 ) )
                    f.write( 'wld2cam Transform', wld2img_h )
                    f.write( 'cam2wld Transform', img2wld_h )
                    f.write( 'ImWidth', self.img_w )
                    f.write( 'ImHeight', self.img_h )
                    f.write( 'CAR_NUM', self.VehicleName)
                    f.write( 'CamFront', int( self.cam2bumper_dist*100 ) )
                    f.write( 'CmFromLeft', int( self.mfromleft*100 ) )
                    f.write( 'CmFromRight', int( self.mfromright*100 ) )
   
                    f.write( "int_calib", self.cam_calib )
                    f.write( "distor", self.distor )
                    f.write( "lat_range", self.lat_range )
                    f.write( "long_range", self.long_range )
                    f.write( "res_per_m", self.res_per_m )
                    f.write( "pitch_calibrated", pitch )
                    f.write(  "yaw_calibrated", yaw )
                    f.write(  "roll_calibrated", float( roll_angle ) )
     
                    f.release()
                    
                    self.calib_finished = True

    def heatmap_angle_check(self):
        accp_votecnt = np.zeros( ( int( self.img_h/2 ), int( self.img_w/2 ) ) )
        
        if len(self.vp_seg_dict_ls) == 0:
            if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
                self.draw_heatmap( accp_votecnt )
            return False

        pitch_ls = np.array(self.pitch_ls); yaw_ls = np.array(self.yaw_ls)
        
        
        median_yaw = np.median(yaw_ls)
        median_pitch = np.median(pitch_ls)
        
        votedict_ls = []
        for votedictidx, ( pitch, yaw, seg_votedict ) in enumerate( zip( pitch_ls, yaw_ls, self.vp_seg_dict_ls) ):
            if median_yaw-self.yaw_filt_lim < yaw < median_yaw+self.yaw_filt_lim and median_pitch-self.pitch_filt_lim < pitch < median_pitch+self.pitch_filt_lim:
                votedict_ls.append(seg_votedict)
        
        #slave = time.time()

        ### Sum vote dict idx:
        # get all index
        xpos_ls = [0]*int(self.inp_w)
        ypos_ls = [0]*int(self.inp_h)
        for votedict in votedict_ls:
            for idx in votedict.keys():
                ypos, xpos = idx.split("_")
                xpos_ls[int(xpos)] += votedict[idx]
                ypos_ls[int(ypos)] += votedict[idx]
                accp_votecnt[ int(ypos), int(xpos) ] += votedict[idx]
        
        #print(time.time() - slave)

        if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
            self.draw_heatmap( accp_votecnt )

        self.heatmap_angle_gen( xpos_ls, ypos_ls )
        
        filt_count = len( votedict_ls )* self.pitch_interval
        
        disk = maindisk + str(self.timestamp) + "/" #ZE
        if config['ENABLE_LOGGING'] == 1 and self.remainingstoragespace > minspace:
            text1 = "\nmedian pitch angles:  " + str(median_pitch) + " median yaw angles:  " + str(median_yaw)
            text2 = "\nvp position:  " + str(self.x_avg) + ' ' + str(self.y_avg)
            text3 = "\nnumber of filtered images:  " + str(filt_count)
            text4 = "\nimages:  " + str(self.total_img_cnt) + "   total pitch: " + str(-1*degrees( self.angles[0] )) + "    yaw: " + str(degrees( self.angles[1] ))
            with open(disk + 'logs.txt', 'a') as t:
                t.write(text1)
                t.write(text2)
                t.write(text3)
                t.write(text4)
                t.close()
        else:
            print( "median pitch angles: ", median_pitch, "median yaw angles: ", median_yaw,   ) #ZE
            print( "vp position: ", self.x_avg, self.y_avg ) #ZE
            print( "number of filtered images: ", filt_count ) #ZE
            print( "images: ", self.total_img_cnt ,"  total pitch:", -1*degrees( self.angles[0] ), "   yaw:" , degrees( self.angles[1] )  ) #ZE
        
        if self.total_img_cnt < self.minFrameCnt: return False
        
        if filt_count < self.filtframes: return False
        else:
            return True
        
    def draw_heatmap(self, accp_votecnt):
        # draw accp votecnt
        if np.max( accp_votecnt ) > 0:
            images = ( ( accp_votecnt/ (np.max( accp_votecnt ) ) )*255 )
        else: images=accp_votecnt
        
        images = np.clip( images, 0, 255 )
        images = images.astype(np.uint8)
        images = cv2.applyColorMap(images, cv2.COLORMAP_JET)
        #cv2.imwrite( "heatmap/{}.jpg".format(self.total_img_cnt +10000000000 ), images ) #ZE
        disk = maindisk + str(self.timestamp) + "/" #ZE
        cv2.imwrite(disk + "{}.jpg".format(self.total_img_cnt +10000000000 ), images )        


    def distest(self):
            self.gen_BEVhom(radians(self.mean_deg), 0 )
            print( "\nsplit" )
            for a,b,c,d in self.bot_kp:
                dist_a = self.getDist([a,b])
                dist_b = self.getDist([c,d])
  
                if 5 < dist_a[1] < 20 and -10 < dist_a[0] < 10:
                    print( dist_a, dist_b )
    
    def heatmap_angle_gen(self, yaw_dist, pitch_dist ):
            self.angles=[-1000,-1000]
            if max(yaw_dist) > 1:
                yaw_max_val = 0
                yaw_max_idx = 0
                for y_cnt in range( int(self.bound_xmin/2), int(self.bound_xmax/2), self.yaw_heatmap_offset ):
                    y_cnt_val = sum( yaw_dist[y_cnt:y_cnt+self.yaw_window] )
                    if yaw_max_val < y_cnt_val: yaw_max_val = y_cnt_val; yaw_max_idx = y_cnt

                pitch_max_val = 0
                pitch_max_idx = 0
                for p_cnt in range( int(self.bound_ymin/2), int(self.bound_ymax/2), self.pitch_heatmap_offset ):
                    p_cnt_val = sum( pitch_dist[p_cnt:p_cnt+self.pitch_window] )
                    if pitch_max_val < p_cnt_val: pitch_max_val = p_cnt_val; pitch_max_idx = p_cnt

                # using the area with the most vp to determine the pitch and yaw angle.
                y_val_total = np.sum( yaw_dist[yaw_max_idx:yaw_max_idx+self.yaw_window] )
                y_sum = 0
                for y_cnt in range( yaw_max_idx, yaw_max_idx+self.yaw_window ):
                    y_sum += y_cnt*yaw_dist[y_cnt]
                yaw_loc = y_sum/y_val_total
                    
                p_val_total = np.sum( pitch_dist[pitch_max_idx:pitch_max_idx+self.pitch_window] )
                p_sum = 0
                for p_cnt in range( pitch_max_idx, pitch_max_idx+self.pitch_window ):
                    p_sum += p_cnt*pitch_dist[p_cnt]
                pitch_loc = p_sum/p_val_total 
                                 
                self.angles = get_py_from_vp( yaw_loc*2, pitch_loc*2, self.cam_calib)
                self.x_avg = yaw_loc*2; self.y_avg = pitch_loc*2
                self.vp_avg = pitch_loc*2

    
    """ Calcylate the intersection of all the lines """
    def calc_intersection(self, filt_line):
        self.all_inters_pt = []
        
        for j in range(len(filt_line)):
          for i in range(j, len(filt_line)):

            if i in [0,1] and j in [0,1]: continue

            for linea in filt_line[i]:
              for lineb in filt_line[j]:
                #linea = filt_line[j]
                #lineb = filt_line[i]

                # change to only allow lines that are not in the same quadrants
                #if lineb[3] == linea[3]: continue
                
                m_b = lineb[0]
                m_a = linea[0]
                
                # Uses filtering to remove lines that are very similar to one another
                #if ( lineb[3] == 2 and linea[3] == 3 ) or ( lineb[3] == 3 and linea[3] == 2 ):
                if abs( m_b - m_a ) < self.angle_gap:
                    continue              
                
                inter_x = (linea[2]-lineb[2])/(m_b - m_a)
                inter_y = m_a*inter_x + linea[2]
                
                if not self.featurepoint_check(inter_x,inter_y): continue
                
                self.all_inters_pt.append( [ inter_x, inter_y ] )
        
    
    
    # generate the BEV homography matrix, the input is in radians
    def gen_BEVhom(self, pitch_norm, yaw_norm):
            #yaw_norm = 0 # set yaw to 0
            yaw_norm = yaw_norm * -1 # need to be flipped since yaw from get_py_from_vp is positive at right side of image, but the homography generation requires yaw to be negative on right side 
            roll_norm = radians(roll_angle)
            print( "pitch, yaw, roll angle:", degrees(pitch_norm), degrees(yaw_norm), degrees(roll_norm) )
            bevres_mat = np.array([ [self.res_per_m, 0,              self.lat_range*self.res_per_m/2],
                                    [0,         -self.res_per_m,     self.long_range*self.res_per_m],
                                    [0,         0,              1] ] )
            

            pitch_mat = np.array([ [1, 0,               0],
                                   [0, cos(pitch_norm), -sin(pitch_norm)],
                                   [0, sin(pitch_norm), cos(pitch_norm)] ] )
            yaw_mat = np.array([ [cos(yaw_norm),   0, sin(yaw_norm)],
                                 [0,        1, 0],
                                 [-sin(yaw_norm),  0, cos(yaw_norm)] ] )
            roll_mat = np.array([ [cos(roll_norm),     sin(roll_norm), 0],
                                  [-sin(roll_norm),    cos(roll_norm), 0],
                                  [0,                   0,               1] ] )

            r_mat = roll_mat @ np.transpose(pitch_mat @ yaw_mat)
            r_mat_cp = np.copy(r_mat)
            
            print( r_mat )
            
            r_mat_cp[0] = r_mat[0]
            r_mat_cp[1] = r_mat[2]
            r_mat_cp[2] = r_mat[1]/(self.cam_h+eps) # if cam_h is 0, this part will fail
            print(bevres_mat,"\n", r_mat_cp,"\n", np.linalg.inv(self.cam_calib) )
            self.hom_mat = bevres_mat @ r_mat_cp @ np.linalg.inv(self.cam_calib)# @ translation_mat
    
    # get distance using object point. obj_pt is list [x,y]
    def getDist(self, obj_pt):
        obj_pt = list(obj_pt)
        # prevent points that are not on the road surface and out of bounds points.
        obj_pt[0] = max( 0, min(obj_pt[0], self.img_h-1))
        obj_pt[1] = max(obj_pt[1], self.vp_calibrated+10)

        coord = Warp( ( obj_pt[0], obj_pt[1] ), self.hom_mat)
        
        long_dist = self.long_range - coord[1]/self.res_per_m
        bump2obj_dist = long_dist - self.cam2bumper_dist
        bump2obj_lat = coord[0]/self.res_per_m - self.lat_range/2  - self.lat_off #ZE #+ self.lat_off 
        
        worldcoord = [ bump2obj_lat, bump2obj_dist ]
        
        return worldcoord
    
    # get image coordinates
    def getPix(self, world_coord):
        coord = list(world_coord)
        
        coord = [ (coord[0] + self.lat_range/2 - self.lat_off )*self.res_per_m, (self.long_range - coord[1] - self.cam2bumper_dist)*self.res_per_m   ] #ZE
        print(coord)
        print( self.hom_mat )
        #coord = [ (coord[0] + self.lat_range/2 + self.lat_off )*self.res_per_m, (self.long_range - coord[1] - self.cam2bumper_dist)*self.res_per_m   ]
        #print( self.long_range - coord[1]/self.res_per_m ) # double checking if the homography is using points based on camera based calibration, world system based on camera
        
        img_point = Warp( coord, np.linalg.inv(self.hom_mat) )
        img_point = ( int(img_point[0]), int(img_point[1]) )
        return img_point
        
        
                        
            
            
        
        







