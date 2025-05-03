#!/usr/bin/env python
# controls argo based on sensors

# topics
# publishes 
# /rudder_sail_cmd,
#   Vector3 with .x=rudder command -1:1=left:right, .y=sail command -1:in, +1:out
# 
# subscribes to 
# /rudder_sail_radio, /human_controlled
# /pose


import rospy
from std_msgs.msg import Float64, Bool, String
from  geometry_msgs.msg import Vector3
import rosparam
import time
from load_or_reload_params import load_or_reload_params
# import numpy as np
from logskip import *

rospy.init_node('control', anonymous=True, log_level=rospy.INFO)
rospy.loginfo('control: initializing')

ARGO_PARAMS="argo.yaml"
load_or_reload_params(ARGO_PARAMS)

RUDDER_FULL_SCALE_DEG=60.0
if rospy.has_param('/argo/rudder_gain'):
    rudder_gain=rospy.get_param('/argo/rudder_gain') #rospy.get_param("rudder_gain")
    rospy.loginfo("rudder_gain=%.2f from argo.yaml",rudder_gain)
else:
    rudder_gain=1
    rospy.logwarn("no rudder_gain in argo.yaml, set rudder_gain=1")
# gain factor from heading error in deg to rudder cmd 
# gain from compass error in degress to -1:+1 range of rudder control

pub_rudder_sail_cmd = rospy.Publisher('rudder_sail_cmd', Vector3, queue_size=10)

rate = rospy.Rate(10)  # main loop rate in Hz


# pub_rudder = rospy.Publisher('cmd_rudder', Float64, queue_size=100)
radio_rudder=None
radio_sail=None
cmd_rudder=None
cmd_sail=None

pose=None
compass=None
target_compass=None


human_control=None
last_human_control=None


def rudder_sail_radio_callback(data):
    global radio_rudder
    global radio_sail
    radio_rudder=data.x
    radio_sail=data.y
    # rospy.logdebug("control: /rudder_sail_radio message %s",data)

def pose_callback(data):
    # control is computed here, on every pose update
    global compass
    global pose
    global target_compass
    global radio_rudder, radio_sail, cmd_rudder, cmd_sail
    global human_control
    pose=data
    compass=pose.z # z component is the magnetic compass heading in deg
    rospy.logdebug("control: compass=%.1f deg",compass)
def human_control_callback(data):
    global human_control, target_compass
    # global target_compass, compass
    # print(data.data)
    human_control=data.data
    if human_control:
        rospy.loginfo("control: human has control")
    elif not target_compass is None:
        rospy.loginfo("control: computer has control to maintain heading %.1f deg",target_compass)
    else:
        rospy.logwarn("control: computer has control but target_compass=None")
import numpy as np

def signed_angle_difference_degrees(angle1_deg, angle2_deg):
    """
    Computes the signed difference between two angles in degrees,
    avoiding the 360-degree wrap-around issue.

    Args:
    angle1_deg: The first angle in degrees (range: -180 to 180).
    angle2_deg: The second angle in degrees (range: -180 to 180).

    Returns:
    The signed difference (angle1_deg - angle2_deg) in degrees,
    within the range of -180 to 180 degrees.
    """
    diff_deg = angle1_deg - angle2_deg
    print(diff_deg)
    return (diff_deg + 180.0) % 360.0 - 180.0

def compute_control():
    global compass
    global target_compass
    global radio_sail, cmd_rudder, cmd_sail
    global human_control
    if human_control or human_control is None:
        target_compass=compass
        if not target_compass is None:
            rospy.logdebug("control: human_control target_compass=%.1f deg",target_compass)
        else:
            rospy.logwarn("control: human_control but target_compass=None")

    if not target_compass is None and not human_control:
        compass_err=signed_angle_difference_degrees(compass,target_compass) # degrees error
        # NOTE will have 2pi-cut problem!
        # we can project to sin and cos components to compute the actual signed angle error

        cmd_rudder=rudder_gain*(compass_err/RUDDER_FULL_SCALE_DEG)
        # full scale rudder cmd +1 maps to approx RUDDER_FULL_SCALE_DEG or 60 deg rudder angle
        # a heading error of 1 deg causes rudder_gain cmd or rudder_gain*60 deg
        # 
        cmd_sail=radio_sail
        rospy.logdebug("control: compass_err=%.1f deg, cmd_rudder=%.2f",compass_err,cmd_rudder)
        pub_rudder_sail_cmd.publish(Vector3(cmd_rudder,cmd_sail,0))

sub_rudder_sail_radio=rospy.Subscriber('/rudder_sail_radio',Vector3,rudder_sail_radio_callback)
sub_compass=rospy.Subscriber('/pose',Vector3,pose_callback)
sub_compass=rospy.Subscriber('/human_controlled',Bool,human_control_callback)
    
def control():
    global rudder_gain
    counter=0
    while not rospy.is_shutdown():
        compute_control()
        counter+=1

        if counter%30==0 and load_or_reload_params(ARGO_PARAMS):
            rudder_gain=rospy.get_param('/argo/rudder_gain') #rospy.get_param("rudder_gain")
            rospy.logdebug("rudder_gain=%.2f from argo.yaml",rudder_gain)

        rate.sleep()


if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        rospy.loginfo('Interrupt: stopping control')



