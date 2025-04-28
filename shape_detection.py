"""
Goal: Detect shapes of objects.
"""
import sys
import io
import time
import picamera
from picamera import PiCamera
import cv2
from picamera.array import PiRGBArray
import numpy as np

def nothing(x):
    #do nothing
    pass


def main():
    print("Hello world!")
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640,480))
    # allow the camera to warmup
    time.sleep(0.1)
    
    # V range: 170+  
    
    # lower boundary RED color range values
    lower1 = np.array([0,40,140])
    upper1 = np.array([20,255,255])
    
    # upper boundary RED color range values
    lower2 = np.array([165, 40,140])
    upper2 = np.array([180,255,255])
    
    # range for color
    lower_red = np.array([0,19, 0])
    upper_red = np.array([180, 255, 255])

    areaThreshold = 0.002
    aspectRatio =1.6
    
    txt_size = 2
    txt_width = 2
    enhanceFactor = 1.2
    
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)
    cv2.createTrackbar("L-S", "Trackbars", 110, 255, nothing)
    cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U-H", "Trackbars", 15, 180, nothing)
    cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)

    font = cv2.FONT_HERSHEY_COMPLEX

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("L-H","Trackbars")
        l_s = cv2.getTrackbarPos("L-S","Trackbars")
        l_v = cv2.getTrackbarPos("L-V","Trackbars")
        u_h = cv2.getTrackbarPos("U-H","Trackbars")
        u_s = cv2.getTrackbarPos("U-S","Trackbars")
        u_v = cv2.getTrackbarPos("U-V","Trackbars")

        lower_red = np.array([l_h,l_s, l_v])
        upper_red = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        kernel = np.ones((5,5), np.uint8)
        maskmask = cv2.erode(mask, kernel)
        
        # draw contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
       
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            # cv2.drawContours(image, [cnt], 0, (0,100,0), 5)
            if area > 400:
                cv2.drawContours(image, [approx], 0, (0,100,0), 5)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                
                text = "None"
            
                if len(approx) == 4:
                    print("It is a rectangle")
                    text = "Rectangle"
                elif len(approx) == 3:
                    print("It is a triange")
                    text = "Triangle"
                    
                elif len(approx) >=10 and len(approx) <20:
                    print("It is a circle")
                    text = "Circle"                    
                    
                cv2.putText(image, text, (x,y), font, 1, (0, 100,0))

        # show the frame
        cv2.imshow("Frame", image)
        cv2.imshow("Mask", mask)
        cv2.imshow("kernel", kernel)
        
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)        
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    

    
if __name__=='__main__':
    main()
