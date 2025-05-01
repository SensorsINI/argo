#!/usr/bin/env python
# controls argo based on sensors

# topics
# publishes rudder_sail_cmd,
#   Vector3 with .x=rudder command -1:1=left:right, .y=sail command -1:in, +1:out
# subscribes to 
# /rudder_sail_radio, /human_controlled
# /compass


import rospy
from std_msgs.msg import Float64, Bool, String
from  geometry_msgs.msg import Vector3

import time
# import numpy as np

rospy.init_node('control', anonymous=True, log_level=rospy.INFO)
rospy.loginfo('setting up argo control')                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            # rospy.init_node('pwm', anonymous=True,log_level=rospy.INFO)

pub_rudder_sail_cmd = rospy.Publisher('rudder_sail_cmd', Vector3, queue_size=10)
# pub_rudder = rospy.Publisher('rudder_cmd', Float64, queue_size=100)
rudder_radio=None
sail_radio=None
pose=None
compass=None
target_compass=None
RUDDER_GAIN=.1
last_human_control=False
human_control=False


def rudder_sail_radio_callback(data):
    rospy.loginfo("/rudder_sail_radio message %s",data)
    radio_rudder=data.x
    radio_sail=data.y
def compass_callback(data):
    global compass
    global pose
    global target_compass
    pose=data
    compass=pose.z
    rospy.logdebug("/compass heading message %s",compass)
    if not target_compass is None:
        compass_err=compass-target_compass
        rudder_cmd=RUDDER_GAIN*compass_err
        rospy.loginfo("compass_err=%.1f deg, rudder_cmd=%.2f",compass_err,rudder_cmd)
        pub_rudder_sail_cmd.publish(Vector3(rudder_cmd,0,0))
def human_control_callback(data):
    global human_control
    global last_human_control
    human_control=data
    rospy.loginfo("human has control: %s",human_control)
    if human_control: # human took control
        if compass is None:
            rospy.logwarn("no /compass to set as reference")
        else:
            target_compass=compass
            rospy.loginfo("human has control with target_compass %.1f deg",target_compass)
    elif last_human_control and not human_control:
        if compass is None:
            logpy.logwarning("no /compass to set as reference")
        else:
            target_compass=compass
            rospy.loginfo("computer took control with target_compass %.1f deg",target_compass)
    last_human_control=human_control



sub_sail=rospy.Subscriber('/rudder_sail_pwm',Vector3,rudder_sail_radio_callback)
sub_compass=rospy.Subscriber('/compass',Vector3,compass_callback)
sub_compass=rospy.Subscriber('/human_controlled',Bool,human_control_callback)

rate = rospy.Rate(10)  # sample rate in Hz


    
def control():
    # while not rospy.is_shutdown():
        # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Interrupt: stopping control')

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        rospy.loginfo('Interrupt: stopping control')



