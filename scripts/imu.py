#!/usr/bin/env python

#Reading the Sensirion SDP32 sensor
#Dev by JJ SlabbertSDP810_example / modified by tobi
#Code tested with Python 2.7
#Run sudo i2cdetect -y 1 in the terminal, to see if the sensor is connected. it will show address 25
#Check the datasheet at https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf
#The sensor i2c address is 0x21 to 0x23 (Not user programable).

import rospy
import RTIMU
import os.path
from  geometry_msgs.msg import Vector3
import smbus
import time
# import numpy as np
import math

 
def imu():

    #np.set_printoptions(precision=4)

    SETTINGS_FILE = "RTIMULib"

    rospy.loginfo("Using IMU settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
      rospy.loginfo("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    rospy.loginfo("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        rospy.logerror("IMU Init Failed")
        sys.exit(1)
    else:
        rospy.loginfo("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    pub_pose = rospy.Publisher('pose', Vector3, queue_size=100)
    pub_accel = rospy.Publisher('accel', Vector3, queue_size=100)
    pub_compass = rospy.Publisher('compass', Vector3, queue_size=100)
    pub_fusion = rospy.Publisher('fusion', Vector3, queue_size=100)
    # x is speed in m/s
    # y is angle in deg
    # z is temperature in celsius
    rospy.init_node('imu', anonymous=True,log_level=rospy.DEBUG)
    rate = rospy.Rate(10) # Hz

    try:
        while not rospy.is_shutdown():
            if imu.IMURead():
                # x, y, z = imu.getFusionData()
                # print("%f %f %f" % (x,y,z))
                data = imu.getIMUData()
                fusionPose = data["fusionPose"]
                rollDeg=math.degrees(fusionPose[0])
                panDeg=math.degrees(fusionPose[1])
                yawDeg=math.degrees(fusionPose[2])
                rospy.logdebug("r: %.2f p: %.2f y: %.2f" % (rollDeg,panDeg,yawDeg)) 
                pub_fusion.publish(Vector3(rollDeg,panDeg,yawDeg))
            else:
                rospy.logwarn('could not read IMU')

            rate.sleep()
    except KeyboardInterrupt:
            rospy.loginfo('Stopping IMU existing continuous measurements')


if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        rospy.loginfo('Stopping IMU existing continuous measurements')
