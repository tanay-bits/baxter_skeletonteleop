#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback

INTER_PATTERN_SLEEP_DURATION = 1

def talker():
    pub = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray)
    rospy.init_node('ledControlTester', anonymous=True)

    led0 = JoyFeedback()
    led0.type = JoyFeedback.TYPE_LED
    led0.id = 0
    led1 = JoyFeedback()
    led1.type = JoyFeedback.TYPE_LED
    led1.id = 1
    led2 = JoyFeedback()
    led2.type = JoyFeedback.TYPE_LED
    led2.id = 2
    led3 = JoyFeedback()
    led3.type = JoyFeedback.TYPE_LED
    led3.id = 3
    rum = JoyFeedback()
    rum.type = JoyFeedback.TYPE_RUMBLE
    rum.id = 0

    while not rospy.is_shutdown():
        msg = JoyFeedbackArray()
        msg.array = [led0, led1, led2, led3, rum]

        rum.intensity = 0.6

        if msg is not None:
            rospy.logdebug("Msg: " + str(msg))
            pub.publish(msg)
            rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

        rum.intensity = 0

        if msg is not None:
            rospy.logdebug("Msg: " + str(msg))
            pub.publish(msg)
            rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
