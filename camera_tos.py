import io
import time
import picamera
from picamera import PiCamera
import sys
sys.path.append('/usr/local/lib/python/cv2/python3.7')
import cv2
from picamera.array import PiRGBArray
import numpy as np
import board
import busio
import time
import adafruit_vl53l0x
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vl53l0x.VL53L0X(i2c,address = 0x29)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480))
rawCapture.truncate(0)

    # start detecting
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv2.imshow("Frame", image)
    print('Range: {}mm'.format(sensor.range - 55))
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    
    time.sleep(0.1)
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
