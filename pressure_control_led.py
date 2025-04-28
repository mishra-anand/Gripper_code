"""
How to run soft rigid - test
with solenoids
Steps:
file_name: change the name after desktop to save the name of the file.
val: array containing all sensor outputs (pressue + bending + tactile)
click green arrow to start code
data collection will commense
use syringe to inflate the gripper and run
If you want to record this run, press ctrl + c on the keyboard to stop the test and record data
If you want to cancel test and not record anything, press red square
After each test, change the file_name argument or you will overwrite existing data
"""
import csv
import board
import busio
import RPi.GPIO as GPIO
i2c = busio.I2C(board.SCL,board.SDA)
import adafruit_ads1x15 .ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
file_name = '/home/master/Desktop/bending2.csv' #change this parameter
ads2 = ADS.ADS1115(i2c, address = 0x48)
ads1 = ADS.ADS1115(i2c, address = 0x49)
ads3 = ADS.ADS1115(i2c, address = 0x4b)

p_max = 13 #change this value to control gripper inflation ammount
v1 = 23
v2 = 24
# m1 = 23
# m2 = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(v1,GPIO.OUT)
GPIO.setup(v2,GPIO.OUT)
# GPIO.setup(m1,GPIO.OUT)
# GPIO.setup(m2,GPIO.OUT)

def get_pressure():
    supply = 5;
    error = 0.1*supply
    voltage = s1.voltage
    pressure = (voltage - error) * (150/(0.8*supply))
    return pressure
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
#blue pos, yellow neg         

def get_sen_values():
    val = [s1.voltage,s2.voltage,s3.voltage,s4.voltage,s5.voltage,s6.voltage,s7.voltage,s8.voltage,s9.voltage,s10.voltage,s11.voltage]
    val2 = [s7.voltage,s8.voltage,s9.voltage,s10.voltage,s11.voltage]  
    return val

sensor_data = []
# for solenoids:
try:
    while True:
        while (get_pressure() <= p_max):
            GPIO.output(v1,GPIO.HIGH) #inflate
            GPIO.output(v2,GPIO.LOW)
            print(str(get_pressure()) + " inflate")
            val = get_sen_values()
            print(val)
            sensor_data.append(val)
            time.sleep(0.5)
        
        #gripper is at specified pressure
        print("hold start")
        print(get_pressure())
        GPIO.output(v1,GPIO.LOW)
        GPIO.output(v2,GPIO.LOW)
        t = 0
        dt=0.1
        while(t < 10):
            time.sleep(dt)
            t +=dt
            val = get_sen_values()
            print(val)
            sensor_data.append(val)
            
        print("deflate")
        print(get_pressure())
        GPIO.output(v1,GPIO.LOW)
        GPIO.output(v2,GPIO.HIGH)
        while(t < 30):
            t +=dt
            val = get_sen_values()
            print(val)
            sensor_data.append(val)
except KeyboardInterrupt:
    print("data collection over")
    with open(file_name ,mode = 'w', newline = '') as file:
        writer = csv.writer(file)
        writer.writerows(sensor_data)
    print("data saved to file")