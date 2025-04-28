import io
import os
import time
import picamera
from picamera import PiCamera
import cv2
from picamera.array import PiRGBArray
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import serial


kernel = np.ones((5,5), np.uint8)
#focal_len = 488.0 # 2022 (using this in experiments)
focal_len = 519.0 #Feb 2023, clibration

#REC 1
#TRUE_SHAPE = "Quadrilateral"
#TRUE_SIZE = 14.4

# CIRCLE 1
#TRUE_SHAPE = "Circle"
#TRUE_SIZE = 15.7

# Triangle 1
#TRUE_SHAPE = "Triangle"
#TRUE_SIZE = 21.0

# CIRCLE 2 (yellow - soft)
#TRUE_SHAPE = "Circle"
#TRUE_SIZE = 15.4

# Triangle 2 (Blue -solid)
#TRUE_SHAPE = "Triangle"
#TRUE_SIZE = 16.4

#REC 2 (yellow - solid)
TRUE_SHAPE = "None" # change for every object ["Triangle", "Circle", "Quadrilateral"] 
TRUE_SIZE = 11.5 # change this value
SAVE_THRESHOLD = 100 # 
SAVE_PATH = "/home/master/Strawberry_RaspberryPi/master/results/ShapeAndSize"
DRAW_CONTOUR = True

class Shapes:
    shape = None
    perimeter = 0
    distance = None
    
    def __init__(self, shape, peri, distance = None):
      self.shape = shape
      self.perimeter = peri
      self.distance = distance     

class Track:
    trackId = 0
    shape = None
    tracks = []
    active = False
    windowSize = 20
    countMissing = 0
    periMean = 0.0
    trackDeathThreshold = 5
    periThresholdPercent = 0.2
    activateThreshold = 5
    distThreshold = 5.0 # in cm. If new detection at farther distance from
    # previous mean, then reject.
    sizeOut = [] # output size of object
    shapeOut = []
    
    def __init__(self, trackId=0):
        self.trackId = trackId
  
    def deactivate(self):
        self.active = False 
        self.shape = None
        self.tracks = []
        self.countMissing = 0
    
    def isActive(self):
        return self.active
    
    def trackUpdate(self, shape, peri, dist=None):
        if (len(self.tracks) ==0):
            self.tracks.append(Shapes(shape, peri, dist))
            self.shape = shape
            self.periMean = peri
            return
            
        if (self.checkConsistency(shape, peri)):
            if len(self.tracks) >= self.windowSize:
                self.tracks.pop(0)
            if self.checkDistanceConsistency(dist):
                self.tracks.append(Shapes(shape, peri, dist))
            else:
                self.tracks.append(Shapes(shape, peri))  
            self.periMean = np.mean([track.perimeter for track in self.tracks])
            self.countMissing = 0
        else:
            self.countMissing +=1

        if self.countMissing >= self.trackDeathThreshold:
            self.deactivate()
        if len(self.tracks) >= self.activateThreshold:
            self.active = True
    
    def assess_performance(self, true_shape, true_size):
        mape = 0.0
        rmse = 0.0
        if len(self.sizeOut) > 0:
            sizError = (np.array(self.sizeOut) - true_size)
            mape = np.mean(np.abs(sizError))/true_size
            rmse = np.sqrt(np.sum(np.power(sizError,2))/len(sizError))
            
            print("MAPE: %.1f " %(100.0*mape))
            print("RMSE: %.2f cm" %(rmse))
            print("Size Std dev: %.2f cm" %(np.std(self.sizeOut)))
            
        if len(self.shapeOut) >0:
            shapeError = np.sum(np.array([shape==true_shape for shape in self.shapeOut]))
            percShapeError = 100.0*shapeError/len(self.shapeOut)
            print("Shape accuracy: %.1f" %(percShapeError))
            
        return mape, rmse
    
    def tracker_out(self):
        """
        Prints states of tracker
        """
        if (self.active):
            print("============================================")
            print("Object shape: %s"%(self.shape))
            print("Mean Perimeter: %.1f pixels" %(self.periMean))
            object_siz = self.get_size()
            
            self.shapeOut.append(self.shape)
            
            if object_siz is not None:
                # print("Object size: %.1f cm" %(object_siz))
                self.sizeOut.append(object_siz)
            print("Object size: %.2f cm" %(np.mean(self.sizeOut)))
                
            return np.mean(self.sizeOut), self.shape, np.std(self.sizeOut)
        
        return 0.0, None, 0.0
            
            
    def checkConsistency(self, shape, peri):
        # check shape consistency
        if self.shape != shape:
            return False
        periError = abs(peri - self.periMean)/self.periMean
        # check size consistency
        if periError > self.periThresholdPercent:
            return False
        
        return True

    def checkDistanceConsistency(self, dist):
        if not dist:
            return False
      
        dist_arr = [track.distance for track in self.tracks if track.distance is not None]
        if len(dist_arr) <= 0:
            return True
      
        mean_dist = np.mean(dist_arr)
      
        if abs(mean_dist - dist) < self.distThreshold:
            return True
        return False
    
    def get_size(self):
        """
        Returns size of the object.
        """
        dist_arr = [track.distance for track in self.tracks if track.distance is not None]
        if len(dist_arr) <= 0: 
            return None
        mean_dist = np.mean(dist_arr)
        return mean_dist*self.periMean/focal_len
      
def detector(image, lower_hsv, upper_hsv):
    """
    Given an image in hsv space, detects objects in it.
    Returns (shape, perimeter)
    """
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image_hsv, lower_hsv, upper_hsv)
    # mask = cv2.erode(mask, kernel, iterations=1)
    # mask = cv2.dilate(mask, kernel, iterations=1)
    
    # morphological operations to remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("Mask", mask)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key = cv2.contourArea)

    if len(contours)<=0:
        return None, 0, mask
    cnt = contours[-1]
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    
    # convex hull
    hull = cv2.convexHull(cnt)
    
    if DRAW_CONTOUR:
        cv2.drawContours(image, [hull], 0, (90, 10, 10), 2)
    
    # polygon epsilon parameter
    # quadrilateral (0.02)
    # triangle (0.1)
    # circle (0.02)
    # tune parameter for shape
    approx = cv2.approxPolyDP(hull, 0.008*perimeter, True) # default 0.02
    [c,r] = cv2.minEnclosingCircle(cnt)
    num_edge = len(approx)
    print("Number of edges: ", num_edge)

    if area > 200:
        shape = "None"
        if num_edge == 4:
            shape = "Quadrilateral"
            if DRAW_CONTOUR:
                cv2.drawContours(image, [approx], 0, (10,10,10), 3)
            
        elif num_edge == 3:
            #print("It is a triange")
            shape = "Triangle"
            if DRAW_CONTOUR:
                cv2.drawContours(image, [approx], 0, (10,10,10), 3)
            
        elif num_edge >=12:
            # print("It is a circle")
            # change this for object category
            shape = "Circle"
            if DRAW_CONTOUR:
                cv2.circle(image, (int(c[0]), int(c[1])), int(r), (10,10,10), 3)
            
        return shape, perimeter, mask
    
    return None, 0, mask

def object_color(image_hsv, mask):
    """
    Given hsv image and mask, gets the range of majority color component.
    """
    # identify pixels corresponding to mask
    pts = np.where(mask==255)
    masked_hsv = image_hsv[pts[0], pts[1]]
    
    # split masked pixels into h, s, and v
    h = masked_hsv[:,0]
    s = masked_hsv[:,1]
    v = masked_hsv[:,2]
    
    # get median estimates
    h_med = np.median(h)
    s_med = np.median(s)
    v_med = np.median(v)
    
    # define bounds based on median estimates
    u_h = min(h_med+20, 179)
    u_s = min(s_med+60, 255)
    u_v = min(v_med+100, 255)
    
    l_h = max(h_med-20,0)
    l_s = max(s_med-60, 0)
    l_v = max(v_med-100, 0)
    
    return np.array([l_h, l_s, l_v]), np.array([u_h, u_s, u_v])

def get_hsv_range(image):
    """
    Looks into the scene and identifies rage of hsv for foreground objects.
    """
    # background has "s" below 50
    lower_hsv_foreground = np.array([0, 50, 0])
    upper_hsv_foreground = np.array([180, 255, 255])
    
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(image_hsv, lower_hsv_foreground, upper_hsv_foreground)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)    

    lower_hsv_update, upper_hsv_update = object_color(image_hsv, mask)

    # time.sleep(10)
    # To Do: Get the median of the mask and put range around it for lower and upper hsv bounds
    #raise NotImplemented
    return lower_hsv_update, upper_hsv_update

def getDistance(volt=None):
    """
    Returns distance in cm.
    """
    maxVal = 3.0
    minVal = 0.5
    distMax = 30.0
    distMin = 5.0
    a = 15.836
    b = 1.16
    if volt is None:
        return -1
    elif volt<minVal:
        return distMax
    elif volt>maxVal:
        return distMin
    
    #return (9.66*(volt**2) -31.4*volt + 30.64)
    # return a*(volt**(-b)) # 2022 calibration
    x = (volt-0.1088)/10.514
    return (1.0/x) - 0.42

def save_data(mask, image, dir_path):
    """
    Save mask and image, given path to folder. 
    """
    print("Saving data...")
    image_name = "color_image.png"
    mask_name = "mask.png"
    cv2.imwrite(os.path.join(dir_path, image_name), image)
    cv2.imwrite(os.path.join(dir_path, mask_name), mask)
    
    print("Finished saving data!")
    
def background_subtract_hsv(image, factor=1.0):
    """
    Identifies the background color and provides bounds for
    HSV so that background can be removed.
    """

    # background subtraction (assuming near-white background)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsvArr = np.einsum('ijk->kij', hsv)
    
    # extract h, s, and v
    hPoints = hsvArr[0][:][:]
    sPoints = hsvArr[1][:][:]
    vPoints = hsvArr[2][:][:]

    # do statistics on HSV points
    (w,h) = np.shape(hPoints)
    hPoints = hPoints.reshape((w*h),)
    sPoints = sPoints.reshape((w*h),)
    vPoints = vPoints.reshape((w*h),)
    
    meanS = np.mean(sPoints)
    sigmaS = np.std(sPoints)
    
    # background removal
    # 0.7 factor best for quadrilateral
    sThreshold = int(meanS + factor*sigmaS)
    
    # choose lower and upper based on saturation value
    lowerHSV = np.array([0, sThreshold, 0])
    upperHSV = np.array([180, 255, 255])
    
    return  lowerHSV, upperHSV

def main():
    # create data sotorage folder
    if not os.path.exists(SAVE_PATH):
        raise FileNotFoundError("SAVE_PATH is incorrectly set")
    
    num_files = len(os.listdir(SAVE_PATH))
    save_path_file = os.path.join(SAVE_PATH, ("%d_%s"%(num_files,TRUE_SHAPE)))
    os.mkdir(save_path_file)
        
    print("Starting detector!")
    
    # initiate camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640,480))
    rawCapture.truncate(0)
    # allow the camera to warmup
    #time.sleep(0.1)
    
    lower_hsv = np.array([170,100,140])
    upper_hsv = np.array([180,255,255])
    
    # depth data from arduino
    dt = 0.05
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=dt)

    ser.reset_input_buffer()
    counter = 0
    
    # initiate tracker
    tracker = Track()

    # start detecting
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        line = ser.readline().decode('utf-8').rstrip()

        # line = ser.readline().decode('utf-8').rstrip()
        dist = None
        line = line.split()
        
        if line and len(line) > 0:
            volt = np.float32(line[0])
            dist = getDistance(volt)
            print("~~~~~~~~~~Object at: %.1f cm"%(dist))

        # background subtraction (assuming near-white background)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsvArr = np.einsum('ijk->kij', hsv)
        
        # extract h, s, and v
        hPoints = hsvArr[0][:][:]
        sPoints = hsvArr[1][:][:]
        vPoints = hsvArr[2][:][:]

        # do statistics on HSV points
        (w,h) = np.shape(hPoints)
        hPoints = hPoints.reshape((w*h),)
        sPoints = sPoints.reshape((w*h),)
        vPoints = vPoints.reshape((w*h),)
        
        meanS = np.mean(sPoints)
        sigmaS = np.std(sPoints)
        
        # background removal
        # 0.7 factor best for quadrilateral
        # 
        sThreshold = int(meanS + 1.0*sigmaS)
        
        # choose lower and upper based on saturation value
        lowerHSV = np.array([0, sThreshold, 0])
        upperHSV = np.array([180, 255, 255])
        
        detect_shape, detect_peri, mask = detector(image, lowerHSV, upperHSV)
        # detect_shape, detect_peri = detector(image, lower_hsv, upper_hsv)
        if not tracker.isActive() and detect_shape is None:
            lower_hsv, upper_hsv = get_hsv_range(image)
            rawCapture.truncate(0)
            continue
        
        if detect_shape is not None:
            tracker.trackUpdate(detect_shape, detect_peri, dist)
            object_size, object_shape, object_siz_std = tracker.tracker_out()
            mape, rmse = tracker.assess_performance(TRUE_SHAPE, TRUE_SIZE)
            object_perimeter_pixel = tracker.periMean
            # mape_text = "MAPE: %.2f"%(mape)
            object_size_text = "%.2f cm"%(object_size)
            object_size_std_text = "%.2f cm"%(object_siz_std)
            
            # write text on image
            cv2.putText(image,object_size_text,(int(0.6*camera.resolution[0]),int(0.25*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (10, 10, 10), 2, cv2.LINE_AA)
            cv2.putText(image,object_shape,(int(0.6*camera.resolution[0]),int(0.18*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (10, 10, 10), 2, cv2.LINE_AA)

            cv2.putText(mask,object_size_text,(int(0.6*camera.resolution[0]),int(0.25*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 10, 10), 2, cv2.LINE_AA)
            cv2.putText(mask,object_shape,(int(0.6*camera.resolution[0]),int(0.18*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 10, 10), 2, cv2.LINE_AA)

        # show the frame
        cv2.imshow("Frame", image)
        cv2.imshow("Mask", mask)
        
        if counter >= SAVE_THRESHOLD:
            save_data(mask, image, save_path_file)
            break
        else:
            counter +=1
        
        
        key = cv2.waitKey(1) & 0xFF             
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

    cv2.destroyAllWindows()        
    """    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        counter+=1
        print("Frame %d\n"%(counter))
        image = frame.array
        line = ser.readline().decode('utf-8').rstrip()
        
        
        detect_shape, detect_peri = detector(image, lower_hsv, upper_hsv)
        if not tracker.isActive() and detect_shape is None:
            lower_hsv, upper_hsv = get_hsv_range(image)
            continue
        
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        print("Converted to hsv!")
        
        hsv_arr = np.einsum('ijk->kij', image_hsv)
        print("Manipulated the array")
        
        h_arr = hsv_arr[0][:][:]
        s_arr = hsv_arr[1][:][:]
        v_arr = hsv_arr[2][:][:]
        print("Extracted H values")

        (w,h) = np.shape(h_arr)
        h_arr = h_arr.reshape((w*h,))
        s_arr = s_arr.reshape((w*h,))
        v_arr = v_arr.reshape((w*h,))
        print("Reshaped h_arr and now creating Histogram")
        
        fig0 = plt.figure(figsize = (10,7))
        plt.hist(h_arr, density=True, bins=180, range=(0,179))
        plt.xlabel("Hue")
        plt.ylabel("Probability")

        fig1 = plt.figure(figsize = (10,7))
        plt.hist(s_arr, density=True, bins=256, range=(0,255))
        plt.xlabel("Saturation")
        plt.ylabel("Probability")
        
        fig2 = plt.figure(figsize = (10,7))
        plt.hist(v_arr, density=True, bins=256, range=(0,255))
        plt.xlabel("Value")
        plt.ylabel("Probability")        
        plt.show()
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        
        
        time.sleep(0.5)
    """

    
if __name__=='__main__':
    main()