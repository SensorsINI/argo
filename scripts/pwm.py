# /usr/bin/env python
# captures and publishes the rudder and sail winch servo positions as pulse width in ms
# (used to be duty cycle which doesn't make sense for a servo)

# topics
# /rudder rudder pulse width in ms, right/starboard is smaller value, left/port is higher value
# /sail position, in smaller, out larger

import rospy
from std_msgs.msg import Float64
import serial
# import RPi.gpio as gpio
import pigpio
import time
import numpy as np


def getTimex():
    return time.time()


pub_sail = rospy.Publisher('sail', Float64, queue_size=100)
pub_rudder = rospy.Publisher('rudder', Float64, queue_size=100)
# rospy.init_node('pwm', anonymous=True,log_level=rospy.INFO)
rospy.init_node('pwm', anonymous=True, log_level=rospy.DEBUG)
rate = rospy.Rate(10)  # sample rate in Hz

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

gpio = pigpio.pi()
if not gpio.connected:
    rospy.logerror('pigpiod is not running')
    quit(1)
else:
    rospy.loginfo('pigpiod is running')

gpio.set_mode(GPIO_RUDDER_RADIO, pigpio.INPUT)
gpio.set_mode(GPIO_SAIL_RADIO, pigpio.INPUT)
if MIRROR_RADIO_TO_SERVO:
    gpio.set_mode(GPIO_RUDDER_SERVO, pigpio.OUTPUT)
    gpio.set_mode(GPIO_SAIL_SERVO, pigpio.OUTPUT)

    gpio.set_PWM_frequency(GPIO_RUDDER_SERVO,50)
    gpio.set_PWM_frequency(GPIO_SAIL_SERVO,50)

    pwm_sail_freq_actual=gpio.get_PWM_frequency(GPIO_SAIL_SERVO)
    rospy.loginfo('servo PWM frequency is %.3f',pwm_sail_freq_actual)
else:
    rospy.logwarn('radio PWM is *not* being mirrored to servos')

upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]


def gpio_callback(gpio, level, tick):
    i = inPINS.index(gpio)
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


gpio.callback(inPINS[0], pigpio.EITHER_EDGE, gpio_callback)
gpio.callback(inPINS[1], pigpio.EITHER_EDGE, gpio_callback)

rospy.loginfo("PWM initialization completed")


def pwm():
    try:
        while not rospy.is_shutdown():
            radio_pwm_sail_ms=np.median(deltaTimes[0])
            radio_pwm_rudder_ms=np.median(deltaTimes[1])
            # debug, can comment out later
            if radio_pwm_sail_ms>.5 and radio_pwm_sail_ms <2.5 and radio_pwm_rudder_ms >.5 and radio_pwm_rudder_ms<2.5:
                outlier=''
            else:
                outlier='**** outlier'
            pub_sail.publish(radio_pwm_sail_ms)
            pub_rudder.publish(radio_pwm_rudder_ms)
            mirrored=''
            if MIRROR_RADIO_TO_SERVO and outlier=='':
                gpio.set_servo_pulsewidth(GPIO_RUDDER_SERVO,int(radio_pwm_sail_ms*1000))
                gpio.set_servo_pulsewidth(GPIO_SAIL_SERVO,int(radio_pwm_rudder_ms*1000))
                mirrored='mirrored'
            rospy.logdebug("pulse widths: sail: %.3f ms, rudder: %.3f ms %s %s",
                radio_pwm_sail_ms,radio_pwm_rudder_ms,outlier,mirrored)
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo('KeyboardInterrupt: stopping gpio')
        gpio.stop()


if __name__ == '__main__':
    try:
        pwm()
    except rospy.ROSInterruptException:
        rospy.loginfo('Interrupt: stopping gpio')
        gpio.stop()
