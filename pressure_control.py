"""
Pressure control code
Inflate gripper to certain pressure, holds, then releases it
Aravind Ramaswami
"""
import board
import busio
i2c = busio.I2C(board.SCL,board.SDA)
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time           

ads1 = ADS.ADS1115(i2c, address = 0x49)
p = AnalogIn(ads1,ADS.P0)
#pos = AnalogIn(ads1,ADS.P0)
p_max = 12 #change this value to control gripper inflation ammount
s1 = 23
s2 = 24
# m1 = 23
# m2 = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(s1,GPIO.OUT)
GPIO.setup(s2,GPIO.OUT)
# GPIO.setup(m1,GPIO.OUT)
# GPIO.setup(m2,GPIO.OUT)

def get_pressure():
    supply = 5;
    error = 0.1*supply
    voltage = p.voltage
    pressure = (voltage - error) * (150/(0.8*supply))
    return pressure

# for solenoids:

while True:
    while (get_pressure() <= p_max):
        GPIO.output(s1,GPIO.HIGH) #inflate
        GPIO.output(s2,GPIO.LOW)
        print(str(get_pressure()) + " inflate")
        time.sleep(0.1)
    
    #gripper is at specified pressure
    print("hold start")
    print(get_pressure())
    GPIO.output(s1,GPIO.LOW)
    GPIO.output(s2,GPIO.LOW)
    time.sleep(20)
#     GPIO.output(m1,GPIO.HIGH)
#     GPIO.output(m2,GPIO.LOW)
    #deflate the gripper
    print("deflate")
    print(get_pressure())
    GPIO.output(s1,GPIO.LOW)
    GPIO.output(s2,GPIO.HIGH)
#     GPIO.output(m1,GPIO.LOW)
#     GPIO.output(m2,GPIO.LOW)
    time.sleep(40)
        
    