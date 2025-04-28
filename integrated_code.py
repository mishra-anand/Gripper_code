#general tips in the code
# sudo i2cdetect -y 1 checks all the i2c connections
#there should be outputs at 14,29,48,49,68
# 14,68 is for pijuice
#29 - TOS sensor
#48,49 are ads boards (48 is one currently connected to pressure sensor)

#the following imports are neccessary for the code to work
import io
import os
import time #to use delay and time.sleep() commands
#This section of commands is neccessary for the computer vision section of the code
import picamera
from picamera import PiCamera
import cv2
from picamera.array import PiRGBArray
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import serial
from general_detector import Shapes, Track
from general_detector import background_subtract_hsv, detector
#This section is neccessary to use the ADS boards, distance sensor
import board
import busio
import adafruit_vl53l0x
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
#This section is neccessary to use GPIO pins
import RPi.GPIO as GPIO #use this import line from raspberry pi
import time

TRACK_TIME = 9 # in seconds 
DETECTION_TO_GRASP_TIME = 10 # in seconds
DIR_PATH = "/home/master/Strawberry_RaspberryPi/master/output" # location to save data
kernel = np.ones((5,5), np.uint8)

class RotationModule:
    
    i2c = busio.I2C(board.SCL,board.SDA) #initializes that we are going to use i2c pins (SCL and SDA on the pi)
    
    #GPIO pins 23 and 24 control the motor
    m1 = 23 # enable
    m2 = 24 # phase
    
    #setup gpio pins as output
    #if you wanted to set them up as input, you would type GPIO.setup(m1,GPIO.IN)
    GPIO.setup(m1,GPIO.OUT) 
    GPIO.setup(m2,GPIO.OUT)
    
    def move_forward(self, dt=5.0):
        print("Rotating forward...")
        # forward
        #Setting the gpio pins to high or low
        #GPIO.output(x,GPIO.HIGH) sets GPIO pin x to high
        GPIO.output(m1,GPIO.HIGH)
        GPIO.output(m2,GPIO.LOW)
        time.sleep(dt)
        
    def reverse(self, dt=5.0):
        print("Rotating reverse...")
        # reverse
        GPIO.output(m1,GPIO.HIGH)
        GPIO.output(m2,GPIO.HIGH)
        time.sleep(dt)
        
    def turn_off(self):
        print("Turn off rotation!")
        # OFF
        GPIO.output(m1,GPIO.LOW)
        GPIO.output(m2,GPIO.LOW)

class TouchSensor:
    """
    Identifies the signal from touching an object.
    """
    i2c = busio.I2C(board.SCL, board.SDA) #initializes i2c
    ads2 = ADS.ADS1115(i2c, address = 0x48)
    ads1 = ADS.ADS1115(i2c, address = 0x49)
    ads3 = ADS.ADS1115(i2c, address = 0x4b)

    s1 = AnalogIn(ads1,ADS.P0) 
    s2 = AnalogIn(ads1,ADS.P1)
    s3 = AnalogIn(ads1,ADS.P2)
    s4 = AnalogIn(ads1,ADS.P3)                                              
    s5= AnalogIn(ads2,ADS.P0) 
    s6 = AnalogIn(ads2,ADS.P1)
    s7 = AnalogIn(ads2,ADS.P2)
    s8 = AnalogIn(ads2,ADS.P3)
    s9 = AnalogIn(ads3,ADS.P0)
    s10 = AnalogIn(ads3,ADS.P1)        
    s11 = AnalogIn(ads3,ADS.P2)
    s12 = AnalogIn(ads3,ADS.P3)

        
    def get_touch_data(self):
        """
        Gets touch sensor reading and outputs an array of the data..
        """
        return TouchSensor.calibrate_touch_data([self.s2.voltage, self.s3.voltage, self.s4.voltage, self.s5.voltage, self.s6.voltage,self.s7.voltage,self.s8.voltage,self.s9.voltage,self.s10.voltage,self.s11.voltage])
    
    @staticmethod
    def calibrate_touch_data(data):
        """
        Calibrates voltage into appropriate units for touch sensor.
        """
        calibrated_output = [TouchSensor.calibration_fun(d) for d in data]
        return calibrated_output
    
    @staticmethod
    def calibration_fun(x):
        """
        defines the y = f(x)
        """
        # To be implemented
        return x

class TOFSensor:
    """
    Time of flight sensor. This works.
    """
    i2c = busio.I2C(board.SCL, board.SDA) #initializes i2c
    sensor = adafruit_vl53l0x.VL53L0X(i2c,address = 0x29) #This tells the pi that the sensor is at 0x29
    OFFSET = 62
    
    def get_distance(self):
        """
        Returns distance in mm.
        """
        return TOFSensor.sensor.range - TOFSensor.OFFSET
    
class PressureSensor:
    i2c = busio.I2C(board.SCL, board.SDA) #initializes that we are going to use i2c pins (SCL and SDA on the pi)
    #sets address of ads board
    # if you want to setup the other module
    # ads2 = ADS.ADS1115(i2c,address = 0x48)
    # to read analog inputs, you have to use AnalogIn() function
    #for example, to read input from A2 on ADS 1, you would do x = AnalogIn(ads1,ADS.P2)
    #to get the voltage ouput, can do x.voltage
    ads1 = ADS.ADS1115(i2c, address = 0x49) 
    pressure = AnalogIn(ads1,ADS.P0)
    pressure_max = 13 #change this value to control gripper inflation ammount
    #GPIO pins for the solenoid valves
    
    # digital pins from raspberry pi
    ser1 = 23 #this one corresponds to the solenoid with 3 tubes in it
    ser2 = 24 #this one corresponds to solenoid with one tube in it
    #The solenoids have 3 states in them
    # s1 = high, s2 = Low -> inflation
    # s1 = Low, s2 = Low -> hold (this will hold even if power is removed from the system)
    # s1 = Low, s2  = High -> deflate
    # this sets up the GPIO layout. You need this command to work.
    #There are different layouts, but use BCM
    #https://www.etechnophiles.com/wp-content/uploads/2021/01/R-Pi-4-GPIO-Pinout-768x572.jpg -> this link is the pin layout corresponding to BCM
    GPIO.setmode(GPIO.BCM)
    #sets up the two gpio pins as output
    GPIO.setup(ser1,GPIO.OUT)
    GPIO.setup(ser2,GPIO.OUT)
    delta_t = 0.1 # time for determining sleep setting
    
    # touch sensing
    touch_sensor = TouchSensor()
    
    # distance sensing
    dist_sensor = TOFSensor()
    
    # create data buffer to write in file
    data_buffer = []
    
    def get_pressure(self, start_time = 0, touch_sense=False, dist_sense=False):
        """
        inputs:
            start_time: start time in ms
            touch_sense: Boolean value to determine whether we want to get touch sensor. 
            dist_sense: Boolean value to determine whether we want to get distance sensor. 
        output:
            pressure: 
            touch data: 
        Returns the pressure in the gripper
        Supply - this is VCC voltage (so 5v because pi outputs 5 volts
        """
        time_stamp = int(1000*(time.time() - start_time))
        supply = 5;
        error = 0.1*supply
        voltage = self.pressure.voltage 
        pressure = (voltage - error) * (150/(0.8*supply)) #formula to get pressure from voltage
        curr_data = [time_stamp, pressure]
        
        if touch_sense:
            touch_data = self.touch_sensor.get_touch_data()
            curr_data += touch_data[:]

        if dist_sense:
            dist_data = self.dist_sensor.get_distance()
            curr_data += [dist_data]   

        self.data_buffer.append(curr_data)
        
        return curr_data
    
    def inflate(self, start_time=0):
        """
        inputs:
            start_time: start time in ms
            touch_sense: Boolean value to determine whether we want to get touch sensor. 
        output:
            Inflate the gripper.
        """
        print("Inflating...")
        # inflate the gripper while the pressure is below the maximum ammount
        #check the pressure every 0.1 seconds
        pressure_data = self.get_pressure(start_time = start_time, touch_sense=True)
        
        while (pressure_data[1]< self.pressure_max):
            # print("Current pressure: %.2f"%(self.get_pressure()))
            # print("Touch sensor reading: ", self.touch_sensor.get_touch_data())
            GPIO.output(PressureSensor.ser1,GPIO.HIGH)
            GPIO.output(PressureSensor.ser2,GPIO.LOW)
            time.sleep(0.1)
            pressure_data = self.get_pressure(start_time=start_time, touch_sense=True)
            
        print("Finished inflating...")

    def deflate(self, start_time=0):
        """
        Deflate the gripper.
        """
        print("Leaving object...")
        GPIO.output(PressureSensor.ser1,GPIO.LOW)
        GPIO.output(PressureSensor.ser2,GPIO.HIGH)
        #GPIO.output(PressureSensor.m1,GPIO.LOW)
        #GPIO.output(PressureSensor.m2,GPIO.LOW)
        self.get_pressure(start_time=start_time)
        
    
    def hold(self, start_time=0, hold_time=15.0):
        """
        Hold the gripper pressure for dt seconds
        """
        print("Starting hold...")
        GPIO.output(PressureSensor.ser1,GPIO.LOW)
        GPIO.output(PressureSensor.ser2,GPIO.LOW)

        # instead of sleep use a for loop to print touch data till "dt" time has passed.
        t = 0
        dt=0.1
        
        while(t < hold_time):
            self.get_pressure(start_time=start_time, touch_sense=True)
            time.sleep(dt)
            t +=dt
            
        print("Finished hold...")
        
    def write_data(self, file_path=None):
        """
        Saves data into .txt file.
        inputs:
            file_path : file location
        output:
            None
        """
        with open(file_path, "w") as file:
            for line in self.data_buffer:
                data_str = "%d"%(line.pop(0))
                
                while len(line) > 0:
                    data_str += (", %.4f"%(line.pop(0)))
                #if len(line) >=7:
                #    data_str = "%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f"%(line[0], line[1], line[2], line[3], line[4], line[5], line[6]) 
                file.write(data_str + "\n")
        print("Finished writing data in file...")            
        
      
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
    
def save_color(image, lower_hsv, upper_hsv, dir_path):
    print("Saving color data...")
    
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_arr = np.einsum('ijk->kij', image_hsv)
    
    h_arr = hsv_arr[0][:][:]
    s_arr = hsv_arr[1][:][:]
    v_arr = hsv_arr[2][:][:]

    (w,h) = np.shape(h_arr)
    h_arr = h_arr.reshape((w*h,))
    s_arr = s_arr.reshape((w*h,))
    v_arr = v_arr.reshape((w*h,))
    
    # h_idx
    h_idx = np.logical_and(h_arr>lower_hsv[0], h_arr<upper_hsv[0])

    # s_idx
    s_idx = np.logical_and(s_arr>lower_hsv[1], s_arr<upper_hsv[1])
    
    # v_idx
    v_idx = np.logical_and(v_arr>lower_hsv[2], v_arr<upper_hsv[2])    
    
    common_idx = np.logical_and(h_idx, np.logical_and(s_idx, v_idx))
    
    # get HSV corresponding to mask
    h_mask = h_arr[common_idx]
    s_mask = s_arr[common_idx]
    v_mask = v_arr[common_idx]

    fig0 = plt.figure(figsize = (10,7))
    plt.hist(h_mask, density=True, bins=180, range=(0,179))
    plt.xlabel("Hue")
    plt.ylabel("Probability")
    file_name = os.path.join(dir_path, "hue.png")
    plt.savefig(file_name)
    
    fig1 = plt.figure(figsize = (10,7))
    plt.hist(s_mask, density=True, bins=256, range=(0,255))
    plt.xlabel("Saturation")
    plt.ylabel("Probability")
    file_name = os.path.join(dir_path, "saturation.png")
    plt.savefig(file_name)
    
    fig2 = plt.figure(figsize = (10,7))
    plt.hist(v_mask, density=True, bins=256, range=(0,255))
    plt.xlabel("Value")
    plt.ylabel("Probability")        
    file_name = os.path.join(dir_path, "value.png")
    plt.savefig(file_name)

def main():
    
    print("Starting detector!")
    start_time = time.time()
    
    # initiate camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640,480))
    rawCapture.truncate(0)
    # allow the camera to warmup
    #time.sleep(0.1)
    
    counter = 0
    
    # initiate tracker
    tracker = Track()
    
    # initiate pressure sensor
    pressure_sensor = PressureSensor()
    
    # initiate TOF sensor
    tof_sensor = TOFSensor()
    
    #initiate touch sensor
    #touch_sensor = TouchSensor()
    
    # initiate rotation module
    rotation_mod = RotationModule()
    
    # start detecting
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        # current time stamp in ms
        time_stamp = int(1000* (time.time()-start_time))
        # get touch sensor data
        #touch_data = touch_sensor.get_touch_data()
        #print("Touch data: ", touch_data)
        
        # get distance in mm units
        #distance_data = tof_sensor.get_distance()
        
        # get pressure in psi units
        all_data = pressure_sensor.get_pressure(start_time=start_time, touch_sense=True, dist_sense=True)
        distance_data = all_data[-1]
        
        print("All data: ", all_data)
        print("Distance: %.1f mm" %(distance_data))
        image = frame.array
        lower_hsv, upper_hsv = background_subtract_hsv(image, 0.7)
        
        print("lower_hsv: ", lower_hsv)
        print("upper_hsv: ", upper_hsv)
        print("========================")
        
        # Run detection
        detect_shape, detect_peri, mask = detector(image, lower_hsv, upper_hsv)
        # detect_shape, detect_peri = detector(image, lower_hsv, upper_hsv)
        
        if not tracker.isActive() and detect_shape is None:
            lower_hsv, upper_hsv = get_hsv_range(image)
            rawCapture.truncate(0)
            continue
        
        if detect_shape is not None:
            # get distance in cm
            dist = distance_data/10.0
            
            tracker.trackUpdate(detect_shape, detect_peri, dist)
            object_size, object_shape, object_siz_std = tracker.tracker_out()
            # mape, rmse = tracker.assess_performance(TRUE_SHAPE, TRUE_SIZE)
            object_perimeter_pixel = tracker.periMean
            # mape_text = "MAPE: %.2f"%(mape)
            object_size_text = "%.2f cm"%(object_size)
            object_size_std_text = "%.2f cm"%(object_siz_std)
            
            curr_time = time.time()
            
            # write text on image
            cv2.putText(image,object_size_text,(int(0.6*camera.resolution[0]),int(0.25*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (10, 10, 10), 2, cv2.LINE_AA)
            cv2.putText(image,object_shape,(int(0.6*camera.resolution[0]),int(0.18*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (10, 10, 10), 2, cv2.LINE_AA)

            cv2.putText(mask,object_size_text,(int(0.6*camera.resolution[0]),int(0.25*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 10, 10), 2, cv2.LINE_AA)
            cv2.putText(mask,object_shape,(int(0.6*camera.resolution[0]),int(0.18*camera.resolution[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 10, 10), 2, cv2.LINE_AA)
            
            time_elapsed = curr_time - start_time
            
            # check if time elapsed >TRACK_TIME. If true, start saving data
            # Next move on to grasping
            if (time_elapsed > TRACK_TIME):
                save_data(mask, image, DIR_PATH)
                save_color(image, lower_hsv, upper_hsv, DIR_PATH)
                
                cv2.destroyAllWindows()
                
                break

        cv2.imshow("Frame", image)
        
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        
        time.sleep(0.1)
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)    
    
    print("Moving to target object")
    time.sleep(DETECTION_TO_GRASP_TIME)
    print("Grasping next!")
    # To do
    
    pressure_sensor.inflate(start_time=start_time)
    # move the object or puck the strawberry
    # To do: Motor rotation
    
    # set hold_time
    pressure_sensor.hold(start_time=start_time, hold_time=15.0) # specify hold time in seconds (this can be changed if neccessary to make demo work)
    #rotation_mod.move_forward(5.0) # time to run motor
    #rotation_mod.reverse(5.0) # time to run motor reverse
    #rotation_mod.turn_off()
    
    pressure_sensor.deflate(start_time=start_time)
    
               # save data (pressure and touch)
    pressure_data_path = os.path.join(DIR_PATH, "pressure_touch_data.txt")
    pressure_sensor.write_data(file_path=pressure_data_path)

    
if __name__=='__main__':
    main()