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

    print('initialiing IMU ROS node')
    rospy.init_node('imu', anonymous=True,log_level=rospy.DEBUG)
    #np.set_printoptions(precision=4)

    SETTINGS_FILE = "RTIMULib"

    abs_path=os.path.abspath(SETTINGS_FILE+".ini")
    rospy.loginfo("Using IMU settings file " + abs_path)
    if not os.path.exists(SETTINGS_FILE + ".ini"):
      rospy.loginfo("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    print('Settings are '+str(s))
    imu = RTIMU.RTIMU(s)

    rospy.loginfo("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        rospy.logerr("*************   IMU Init Failed, quiting imu.py")
        quit(1)
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
    rate = rospy.Rate(10) # Hz

    try:
        while not rospy.is_shutdown():
            if imu.IMURead():
                # x, y, z = imu.getFusionData()
                # print("%f %f %f" % (x,y,z))
                data = imu.getIMUData()

                """ 
                sample of data returned

                {'accelValid': True, 'fusionQPoseValid': True, 'timestamp': 1744717994965783L, 
                'compassValid': True, 
                'compass': (-24.0, 20.340002059936523, 2.3400001525878906), (should be uT (micro Tesla)? )
                'accel': (-0.4088134765625, 0.838134765625, 0.34693604707717896), (units shold be g)
                'gyroValid': True, 
                'gyro': (0.0062618679367005825, 0.0140159260481596, -0.004969525150954723), (should be rad/s)
                'fusionQPose': (0.04537138342857361, 0.2931070029735565, -0.5019389986991882, -0.8124573230743408), 
                'fusionPoseValid': True, 
                'fusionPose': (1.2032440900802612, 0.4452976584434509, -2.7216269969940186)} (should be rad)
                """
#                rospy.logdebug(data)
                fusionPose = data["fusionPose"]
                gyro=data['gyro']
                accel=data['accel']
                compass=data['compass']

                rollDeg=math.degrees(fusionPose[0])
                panDeg=math.degrees(fusionPose[1])
                yawDeg=math.degrees(fusionPose[2])
                
                gx=math.degrees(gyro[0])
                gy=math.degrees(gyro[1])
                gz=math.degrees(gyro[2])
                
                ax=accel[0]
                ay=accel[1]
                az=accel[2]

                cx=compass[0]
                cy=compass[1]
                cz=compass[2]

                rospy.logdebug("pose rpy=(%.2f,%.2f,%.2f), gyro xyz=(%.1f,%.1f,%.1f)deg/s, acc xyz=(%.2f,%.2f,%.2f)g, mag xyz=(%.1f,%.1f,%.1f)uT" 
                    % (rollDeg,panDeg,yawDeg,gx,gy,gz, ax,ay,az, cx,cy,cz)) 
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
