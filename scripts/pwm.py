#!/usr/bin/env python
# captures and publishes the rudder and sail winch servo positions as pulse width in ms
# (used to be duty cycle which doesn't make sense for a servo)

# topics

# /rudder_sail_radio, Vector3 with .x=rudder, .y=sail, .z reserved (currently 0)
# values are normalized to -1:+1 assuming range of pulse width is 1ms to 2ms
# rudder -1 means full left rudder (turn CCW looking down on boat), +1 is full right rudddder
# sail -1 means pulled in fully, +1 let out fully

#previously was
# /rudder rudder pulse width in ms, right/starboard is smaller value, left/port is higher value
# /sail position, in smaller, out larger

import rospy
from std_msgs.msg import Float64, Bool
from  geometry_msgs.msg import Vector3 # publish rudder and sail together as x and y. z=0 always

import serial
# import RPi.GPIO as GPIO # standard GPIO library for low level pin control
import pigpio # daemon-based PWM controller with good precision
import time
import numpy as np



def rudder_sail_cmd_callback(data):
    global cmd_rudder
    global cmd_sail
    rospy.loginfo("/rudder_sail_cmd message %s",data)
    cmd_rudder=data.x
    cmd_sail=data.y
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             # rospy.init_node('pwm', anonymous=True,log_level=rospy.INFO)
rospy.init_node('pwm', anonymous=True, log_level=rospy.INFO)
# pub_sail = rospy.Publisher('sail', Float64, queue_size=10)
# pub_rudder = rospy.Publisher('rudder', Float64, queue_size=10)
pub_rudder_sail=rospy.Publisher('rudder_sail_radio', Vector3, queue_size=10)
pub_human_controlled=rospy.Publisher('human_controlled', Bool, queue_size=2)
sub_sail=rospy.Subscriber('/rudder_sail_cmd',Vector3,rudder_sail_cmd_callback)
rate = rospy.Rate(10)  # sample rate in Hz

def rudder_sail_cmd_callback(data):
    pass

# number of samples to median filter over; should be odd number
smoothingWindowLength = 3

MIRROR_RADIO_TO_SERVO=True

# PWM outputs from radio receiver are wired to header pins 18 (rudder) and 22 (sail)
# On RPi3 Model 2B, these are pigpio GPIO pins 24 and 25
# see https://abyz.me.uk/rpi/pigpio/index.html
GPIO_RUDDER_RADIO = 25
GPIO_SAIL_RADIO = 24
GPIO_RUDDER_SERVO = 18
GPIO_SAIL_SERVO = 23
# was [18,23] #pinnumbers that are used(BCM nameingconvention)
inPINS = [GPIO_RUDDER_RADIO, GPIO_SAIL_RADIO]
# was [18,23] #pinnumbers that are used(BCM nameingconvention)
outPINS = [GPIO_RUDDER_SERVO, GPIO_SAIL_SERVO]

HUMAN_CONTROL_TIMEOUT_S=2 
# time in seconds that human takes control after rudder command deviates by
# more than HUMAN_CONTROL_THRESHOLD
HUMAN_CONTROL_THRESHOLD=.1
# threshold deviation from zero for radio rudder input that human takes control

rospy.loginfo("human control timeout=%.1fs, human control threshold=%.1f", HUMAN_CONTROL_TIMEOUT_S, HUMAN_CONTROL_THRESHOLD)

# GPIO.setmode(GPIO.BCM) # use broadcom GPIO (not pin numbers) with GPIO. pigpio always uses GPIO numbers.

ppio = pigpio.pi()
rospy.loginfo("pigpio.get_pigpio_version()=%s"% ppio.get_pigpio_version())
if not ppio.connected:
    rospy.logerror('pigpiod is not running')
    quit(1)
else:
    rospy.loginfo('pigpiod is running')


rospy.loginfo('setting up GPIO radio pins as inputs and servo pin with pull up/down disabled')
ppio.set_mode(GPIO_RUDDER_RADIO, pigpio.INPUT)
ppio.set_mode(GPIO_SAIL_RADIO, pigpio.INPUT)
ppio.set_pull_up_down(GPIO_RUDDER_SERVO,pigpio.PUD_OFF)
ppio.set_pull_up_down(GPIO_SAIL_SERVO,pigpio.PUD_OFF)

if MIRROR_RADIO_TO_SERVO:
    rospy.loginfo('setting servo GPIO as outputs')
    ppio.set_mode(GPIO_RUDDER_SERVO, pigpio.OUTPUT)
    ppio.set_mode(GPIO_SAIL_SERVO, pigpio.OUTPUT)
    ppio.set_pad_strength(0,14) # set all GPIO 0-27 with strength 14mA to drive the servos overriding radio

    ppio.set_PWM_frequency(GPIO_RUDDER_SERVO,50)
    ppio.set_PWM_frequency(GPIO_SAIL_SERVO,50)

    pwm_sail_freq_actual=ppio.get_PWM_frequency(GPIO_SAIL_SERVO)
    rospy.loginfo('servo PWM frequency is %.3f',pwm_sail_freq_actual)
else:
    rospy.logwarn('radio PWM is *not* being mirrored to servos')

upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]


def gpio_callback(ppio, level, tick):
    i = inPINS.index(ppio)
    v = level
    if (v == 0):  # falling edge
        downTimes[i].append(tick)  # tick is in us since boot, wraps every 72m
        if len(downTimes[i]) > smoothingWindowLength: del downTimes[i][0]
        dtMs = 1e-3*(downTimes[i][-1]-upTimes[i][-1])
        if(dtMs > 0):
            deltaTimes[i].append(dtMs)  # delta time in ms
        else:
            rospy.loginfo("tick wrapped around, got negative dtMs")

    else:  # rising edge
        upTimes[i].append(tick)
        if len(upTimes[i]) > smoothingWindowLength: del upTimes[i][0]
    if len(deltaTimes[i]) > smoothingWindowLength: del deltaTimes[i][0]


ppio.callback(inPINS[0], pigpio.EITHER_EDGE, gpio_callback)
ppio.callback(inPINS[1], pigpio.EITHER_EDGE, gpio_callback)

rospy.loginfo("PWM initialization completed")


def pwm():
    time_last_human_cmd=time.time()
    human_has_control=True # set True when boat PWM radio input has priority

    try:
        while not rospy.is_shutdown():
            radio_pwm_sail_ms=np.median(deltaTimes[0])
            radio_pwm_rudder_ms=np.median(deltaTimes[1])
            # debug, can comment out later
            if radio_pwm_sail_ms>.5 and radio_pwm_sail_ms <2.5 and radio_pwm_rudder_ms >.5 and radio_pwm_rudder_ms<2.5:
                outlier=''
            else:
                outlier='**** outlier'
            radio_pwm_rudder_normalized=-(-1+2*(radio_pwm_rudder_ms-1)) 
            # 1ms->+1, full right rudder, 2ms->-1, full left rudder
            radio_pwm_sail_normalized=-1+2*(radio_pwm_sail_ms-1) 
            # 1ms->-1, sail pulled fully in, 2ms->+1, sail fully let out
            pub_rudder_sail.publish(Vector3(radio_pwm_rudder_normalized, radio_pwm_sail_normalized,0))
            if np.abs(radio_pwm_rudder_normalized)>HUMAN_CONTROL_THRESHOLD:
                time_last_human_cmd=time.time()
                if not human_has_control:
                    rospy.loginfo("human took control")
                    pub_human_controlled.publish(True)

            human_has_control_now=time.time()-time_last_human_cmd<HUMAN_CONTROL_TIMEOUT_S
            if not human_has_control_now and human_has_control:
                rospy.loginfo("computer took control")
                pub_human_controlled.publish(False)
            human_has_control=human_has_control_now
            # pub_sail.publish(radio_pwm_sail_ms)
            # pub_rudder.publish(radio_pwm_rudder_ms)
            mirrored=''
            if MIRROR_RADIO_TO_SERVO and outlier=='':
                ppio.set_servo_pulsewidth(GPIO_RUDDER_SERVO,int(radio_pwm_sail_ms*1000))
                ppio.set_servo_pulsewidth(GPIO_SAIL_SERVO,int(radio_pwm_rudder_ms*1000))
                mirrored='mirrored'
            rospy.logdebug("pulse widths: sail: %.3f ms, rudder: %.3f ms %s %s",
                radio_pwm_sail_ms,radio_pwm_rudder_ms,outlier,mirrored)
            rate.sleep()
    finally:
        rospy.loginfo('Finally: stopping ppio and setting servo GPIOs to inputs')
        # if MIRROR_RADIO_TO_SERVO and outlier=='':
        #     ppio.set_servo_pulsewidth(GPIO_RUDDER_SERVO,0)
        #     ppio.set_servo_pulsewidth(GPIO_SAIL_SERVO,0)
        ppio.set_mode(GPIO_RUDDER_SERVO, pigpio.INPUT)
        ppio.set_mode(GPIO_SAIL_SERVO, pigpio.INPUT)
        # ppio.set_pull_up_down(GPIO_RUDDER_SERVO,pigpio.PUD_UP)
        # ppio.set_pull_up_down(GPIO_SAIL_SERVO,pigpio.PUD_UP)
        ppio.stop()
        # GPIO.setup(GPIO_SAIL_SERVO, GPIO.IN)
        # GPIO.setup(GPIO_RUDDER_SERVO, GPIO.IN)
        # GPIO.setup(GPIO_SAIL_SERVO, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        # GPIO.setup(GPIO_RUDDER_SERVO, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        # GPIO.cleanup()

if __name__ == '__main__':
    try:
        pwm()
    except rospy.ROSInterruptException:
        rospy.loginfo('Interrupt: stopping ppio')
        ppio.stop()
