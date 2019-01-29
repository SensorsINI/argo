#!/usr/bin/env python

#Reading the Sensirion SDP32 sensor
#Dev by JJ SlabbertSDP810_example / modified by tobi
#Code tested with Python 2.7
#Run sudo i2cdetect -y 1 in the terminal, to see if the sensor is connected. it will show address 25
#Check the datasheet at https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf
#The sensor i2c address is 0x21 to 0x23 (Not user Programable).

import rospy
from std_msgs.msg import Float64
import smbus
import time
#import numpy as np

# from https://stackoverflow.com/questions/49906101/byte-array-to-int-in-python-2-x-using-standard-libraries
def int_from_bytes(b):
    '''Convert big-endian signed integer bytearray to int

    int_from_bytes(b) == int.from_bytes(b, 'big', signed=True)'''
    if not b: # special-case 0 to avoid b[0] raising
        return 0
    n = b[0] & 0x7f # skip sign bit
    for by in b[1:]:
        n = n * 256 + by
    if b[0] & 0x80: # if sign bit is set, 2's complement
        bits = 8*len(b)
        offset = 2**(bits-1)
        return n - offset
    else:
        return n


# differential presssure in pascal

def windspeedFromDiffPressure(dp) :
    rho=1.1289 # density at calib pressure
    p0=966 # mbar calib pressure
    t0=298.15 # calib temperature
 

#np.set_printoptions(precision=4)

channel1 = rospy.Publisher('anem_data1', Float64, queue_size=10)
channel2 = rospy.Publisher('anem_data2', Float64, queue_size=10)
channel2 = rospy.Publisher('anem_data3', Float64, queue_size=10)
rospy.init_node('anem', anonymous=True)
rate = rospy.Rate(3) # 3hz

rospy.loginfo 'Opening i2c SMBus'
bus=smbus.SMBus(1) #The default i2c bus
i2cAddr=(0x21,0x22,0x23)

rospy.loginfo 'Stopping existing continuous measurements'
for a in i2cAddr:
    bus.write_i2c_block_data(a, 0x3F, [0xF9]) #Stop any cont measurement of the sensor

time.sleep(0.8)

#Start Continuous Measurement (5.3.1 in Data sheet)
rospy.loginfo 'Starting continuous measurement (5.3.1 in Data sheet)'

##Command code (Hex)        Temperature compensation            Averaging
##0x3603                    Mass flow                           Average  till read
##0x3608                    Mass flow None                      Update rate 0.5ms
##0x3615                    Differential pressure               Average till read
##0x361E                    Differential pressure None          Update rate 0.5ms

for a in i2cAddr:
    bus.write_i2c_block_data(a, 0x36, [0X15]) 

time.sleep(.1)

rospy.loginfo("Initialization completed")


def anem():
    dp=list()
    try:
        i=0
        del dp[:]
        for a in i2cAddr:
            b=bus.read_i2c_block_data(a,0,9)
    	v=int_from_bytes([b[0],b[1]])
    	#print(b,v)
        dp.append(v/240.)
        rospy.loginfo("ch1: %f, ch2: %f, ch3: %f",dp(0),dp(1),dp(2))
        # print [" %8.4f" % v for v in dp]
        channel1.publish(dp(0))
        channel2.publish(dp(1))
        channel3.publish(dp(2))
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        anem()
    except rospy.ROSInterruptException:
        pass
