#!/usr/bin/env python

#Reading the Sensirion SDP32 sensor
#Dev by JJ SlabbertSDP810_example / modified by tobi
#Code tested with Python 2.7
#Run sudo i2cdetect -y 1 in the terminal, to see if the sensor is connected. it will show address 25
#Check the datasheet at https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf
#The sensor i2c address is 0x21 to 0x23 (Not user programable).

import rospy
#import roslib
from  geometry_msgs.msg import Vector3
import smbus
import time
import numpy as np

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

#def windspeedFromDiffPressure(dp) :
#    rho=1.1289 # density at calib pressure
#    p0=966 # mbar calib pressure
#    t0=298.15 # calib temperature
# to be completed, just started now.... need to work from datasheet
    

# following from sensirion https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/
  
def calculate_angle_deg(dp1, dp2, dp3):
    # parameter for sinus curve, estimated on one measurement at 7.2 m/s
    b=0.64
    s1 = dp1+dp2
    s2 = dp2+dp3

    if s1!=0:
        g1 = s2/s1
    else:
        g1=1e12
    if g1!=0:
        g2 = 1/g1
    else:
        g2=1e12 # arg large value

    # |g1|==|g2| for omega = 3b/2
    if np.abs(g1)<(3*b/2):
        # lookup based on g1
        magn = 2*np.sqrt(1-g1+g1**2)
        w = np.pi/4+np.arctan((2*g1-1)/np.sqrt(3))-(np.sign(s1)-1)*np.pi/2
    else:
    # lookup based on g2
        w = (np.pi/2-np.arctan((2*g2-1)/np.sqrt(3))-(np.sign(s2)-1)*np.pi/2)
    w=w*(180./np.pi)
    return w

def calculate_speed_mps(dp1,dp2, dp3):
    rho = 1.2
    # scale factor sqrt(2) estimated by measurements
    # A in differential pressure
    A = np.sqrt(2)*(np.abs(dp1) + np.abs(dp2) + np.abs(dp3))
    # A in m/s
    A = np.sqrt(2 * rho * A)
    return A

 
def anem():

    #np.set_printoptions(precision=4)

    pub_diff_pressure = rospy.Publisher('anem_diffpressure', Vector3, queue_size=100)
    pub_wind_temp = rospy.Publisher('anem_speed_angle_temp', Vector3, queue_size=100)
    # x is speed in m/s
    # y is angle in deg
    # z is temperature in celsius
    rospy.init_node('anem', anonymous=True,log_level=rospy.DEBUG)
    rate = rospy.Rate(10) # Hz

    rospy.loginfo('anem: Opening i2c SMBus')
    bus=smbus.SMBus(1) #The default i2c bus
    i2cAddr=(0x21,0x23,0x22) # 21 is CCW, 23 is center, 22 is clockwise (viewed from top)

    rospy.loginfo('anem: Stopping existing continuous measurements')
    for a in i2cAddr:
        bus.write_i2c_block_data(a, 0x3F, [0xF9]) #Stop any cont measurement of the sensor

    time.sleep(0.8)

    #Start Continuous Measurement (5.3.1 in Data sheet)
    rospy.loginfo('anem: Starting 0x3615 continuous measurement with average till read (5.3.1 in Data sheet)')

    ##Command code (Hex)        Temperature compensation            Averaging
    ##0x3603                    Mass flow                           Average  till read
    ##0x3608                    Mass flow None                      Update rate 0.5ms
    ##0x3615                    Differential pressure               Average till read
    ##0x361E                    Differential pressure None          Update rate 0.5ms

    for a in i2cAddr:
        bus.write_i2c_block_data(a, 0x36, [0X15]) 

    time.sleep(.1)

    rospy.loginfo("anem: Initialization of anem wind sensor completed")

    dp=list()
    temps=list()

    try:
        while not rospy.is_shutdown():
            del dp[:]
            del temps[:]
            for a in i2cAddr:
                b=bus.read_i2c_block_data(a,0,9)
                v=int_from_bytes([b[0],b[1]])/240.# convert to Pascals diff pressure
                temp=int_from_bytes([b[3],b[4]])/200. # convert to deg celsius
                dp.append(v)
                temps.append(temp)
            # print [" %8.4f" % v for v in dp]
            pub_diff_pressure.publish(Vector3(dp[0],dp[1],dp[2]))
            angle_deg=calculate_angle_deg(dp[0],dp[1],dp[2])
            speed_mps=calculate_speed_mps(dp[0],dp[1],dp[2])
            temp_celsius=(temps[0]+temps[1]+temps[2])/3;
            pub_wind_temp.publish(Vector3(speed_mps,angle_deg,temp_celsius))
            rospy.logdebug("Anemometer: speed(m/s)=%.2f angle(deg)=%.1f temp(C)=%.1f dp(pascal)=(%.4f, %.4f, %.4f)",speed_mps,angle_deg,temp_celsius, dp[0],dp[1],dp[2])

            rate.sleep()
    except KeyboardInterrupt:
            rospy.loginfo('Stopping existing continuous measurements')
            for a in i2cAddr:
                bus.write_i2c_block_data(a, 0x3F, [0xF9]) #Stop any cont measurement of the sensor

if __name__ == '__main__':
    try:
        anem()
    except rospy.ROSInterruptException:
        rospy.loginfo('Stopping existing continuous measurements')
        for a in i2cAddr:
            bus.write_i2c_block_data(a, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
