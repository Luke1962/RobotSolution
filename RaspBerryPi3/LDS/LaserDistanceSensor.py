
# -*- coding: iso-8859-1 -*-
import RPi.GPIO as GPIO
from SimpleCV import *
import os
#import pytest
from PIL import Image
import numpy as np
from laser_range_finder import LaserRangeFinder, utils
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)

def get_distance(self, off_img, on_img, save_images_dir=None, **kwargs):
        """
        Calculates distance using two images.
        
        Keyword arguments:
        off_img -- a stream or filename of an image assumed to have no laser projection
        on_img -- a stream of filename of an image assumed to have a laser projection
        """
        vert_fov_deg =  41 # Camera's verticial field-of-view in degrees.
        horz_fov_deg =53   # Camera's horizontal field-of-view in degrees.
        ro = 0.21 # Radian offset  
        h=22# Distance between camera center and laser.
        filter_outliers = false
        outlier_filter_threshold = 1
        
        # Normalize image brightness.
        off_img = Image.fromarray(utils.normalize(np.array(off_img)).astype('uint8'), 'RGBA')
        on_img = Image.fromarray(utils.normalize(np.array(on_img)).astype('uint8'), 'RGBA')
        
        # Strip out non-red channels.
        off_img = utils.only_red(off_img)

        on_img = utils.only_red(on_img)

		
        # Calculate difference.
        # The laser line should now be the brightest pixels. 
        diff_img = difference(off_img, on_img)
        
        # Estimate the pixels that are the laser by
        # finding the row in each column with maximum brightness.
        width, height = size = diff_img.size
        x = diff_img.convert('L')
        y = np.asarray(x.getdata(), dtype=np.float64).reshape((x.size[1], x.size[0]))
        laser_measurements = [0]*width # [row]
        laser_brightness = [0]*width # [brightness]
        for col_i in xrange(y.shape[1]):
            col_max = max([(y[row_i][col_i], row_i) for row_i in xrange(y.shape[0])])
            col_max_brightness, col_max_row = col_max
            #print col_i, col_max
            #pix[col_i, col_max_row] = 255
            laser_measurements[col_i] = col_max_row
            laser_brightness[col_i] = col_max_brightness
            
        # Ignore all columns with dim brightness outliers.
        # These usually indicate a region where the laser is absorbed or otherwise scattered
        # too much to see.
        if filter_outliers:
            brightness_std = np.std(laser_brightness)
            brightness_mean = np.mean(laser_brightness)
            outlier_level = brightness_mean - brightness_std * outlier_filter_threshold
        final_measurements = [-1]*width # [brightest row]
        for col_i, col_max_row in enumerate(laser_measurements):
        
            if save_images_dir:    
                pix1[col_i, col_max_row] = 255
                    
            if not filter_outliers \
            or (filter_outliers and laser_brightness[col_i] > outlier_level):
                if save_images_dir:
                    pix2[col_i, col_max_row] = 255
                    
            # Assuming the laser is mounted below the camera,
            # we can assume all points above the centerline are noise.
            if col_max_row < height/2:
                continue
                
            if not filter_outliers \
            or (filter_outliers and laser_brightness[col_i] > outlier_level):
                if save_images_dir:
                    pix3[col_i, col_max_row] = 255
                final_measurements[col_i] = col_max_row
        
 
 
        #https://sites.google.com/site/todddanko/home/webcam_laser_ranger
        #https://shaneormonde.wordpress.com/2014/01/25/webcam-laser-rangefinder/
        
        #https://www.raspberrypi.org/documentation/hardware/camera.md
        #Horizontal field of view     53.50 +/- 0.13 degrees
        #Vertical field of view     41.41 +/- 0.11 degress
        
        # h = distance between laser and camera in mm
        
        # D = h/tan(theta)
        # theta = pfc*rpc + ro
        
        # pfc = ? # number of pixels from center of focal plane
        # Calculated per pixel.
        
        # rpc = ? # radians per pixel pitch
        # 180 deg=pi rad
        rpc = (vert_fov_deg*pi/180.)/height
        
        # ro = ? # radian offset (compensates for alignment errors)
        
        # Convert the pixel measurements to distance.
        D_lst = []
        for laser_row_i in final_measurements:
            if laser_row_i < 0:
                # No laser could be detected in this column.
                D_lst.append(laser_row_i)
            else:
                pfc = abs(laser_row_i - height)
                D = h/tan(pfc*rpc + ro)
                D_lst.append(D)
            
        return D_lst



##def only_red(im):
##    """
##    Strips out everything except red.
##    """
##    im = im.convert("RGBA")
##    data = np.array(im)
##    red, green, blue, alpha = data.T
##    im2 = Image.fromarray(red.T)
##    return im2

# MAIN -----------------------------------------------------------

#cam = Camera(prop_set={'width':320,'height':240})
cam = Camera(prop_set={'width':640,'height':480})
threshold = 5.0 # if mean exceeds this amount do something

while True:
    # acquisizione con LASER OFF---------------------------
    prev = cam.getImage()
    

    # acquisizione con LASER ON---------------------------    
    GPIO.output(7,True) # LASER ON    
    time.sleep(0.2) #wait for half a second    
    curr = cam.getImage() #acquisizione     
    GPIO.output(7,False)# LASER OFF
    time.sleep(0.1)    
    #------------------------------------------------------

    
    curr.drawText('curr')
    curr.show()
    time.sleep(0.5)
    curr.equalize().show()

    
    # differenza ------------------------------------------
    diff = (curr - prev )

    #elaborazione -----------------------------------------
    imgChannels = diff.splitChannels(0)
    #b,g,r = cv2.split(diff)
    imgChannels[0].drawText('red_channel')
    imgChannels[0].show()
    
    #diff1=only_red(diff)
    
    diff1 = diff.erode().binarize(10).invert() 
    
    diff1.drawText('diff1')
##    for p in diff:
##        p.draw(color=Color.RED,normalize=False)

    #visualizzazione -----------------------
    diff1.show()
   
##    motion = curr.findMotion(prev)
##
##    motion.show()
##    
##    
##    for m in motion:
##        m.draw(color=Color.AZURE,normalize =true)
##
##       
##        diff.show()
##        
##
##    matrix = diff.getNumpy()
##    
##    mean = matrix.mean() #luminosita' media
##    
##    print (mean)
##    if mean >= threshold:
##        print ("Motion Detected")
## 
##################################
#   lines = img.findLines()

#    lines.draw(Color.RED) #outline the line segments in red
 
#      img = cam.getImage()
 
#    histogram = diff.histogram(255)
#    plot(histogram)
    #diff = Image.fromarray(utils.normalize(np.array(diff)).astype('float'), 'RGBA')


def compress_list(lst, bins=10):
    """
    Averages a list of numbers into smaller bins.
    """
    new_lst = []
    chunk_size = int(round(len(lst)/float(bins)))
    for bin in xrange(bins):
        samples = lst[bin*chunk_size:bin*chunk_size+chunk_size]
        new_lst.append(sum(samples)/float(len(samples)))
    return new_lst



def normalize(arr):
    """
    Linear normalization
    http://en.wikipedia.org/wiki/Normalization_%28image_processing%29
    """
    arr = arr.astype('float')
    # Do not touch the alpha channel
    for i in range(3):
        minval = arr[...,i].min()
        maxval = arr[...,i].max()
        if minval != maxval:
            arr[...,i] -= minval
            arr[...,i] *= (255.0/(maxval-minval))
    return arr


#------------------------------------------------------------------

