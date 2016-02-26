#!/usr/bin/env python
import rospy
import tf
import baxter_interface
from geometry_msgs.msg import Point
from skeletonmsgs_nu.msg import Skeletons
from baxter_interface import CHECK_VERSION
from baxter_skeletonteleop.limb_mover import LimbMover

class Teleop:
    def __init__(self):
        rospy.init_node('teleop')
        self.tflistener = tf.TransformListener()
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.rs.state().enabled:
            rospy.logerr("Baxter not enabled")

        self.mover_left = LimbMover("left")
        self.mover_right = LimbMover("right")
        
        # subscribe to /skeletons topic and perform callback
        rospy.Subscriber("/skeletons", Skeletons, self.callback)

    def callback(self, message):
        number_users = len(message.skeletons)
        user = message.skeletons[number_users-1].userid
        # print str(user)
        trans_pointR = Point()
        trans_pointL = Point()

        try:
            (transR, rotR) = self.tflistener.lookupTransform('/torso_' + str(user),
                '/right_hand_' + str(user), rospy.Time(0))
            (transL, rotL) = self.tflistener.lookupTransform('/torso_' + str(user),
                '/left_hand_' + str(user), rospy.Time(0))
            # rospy.loginfo("\ntrans_x: %s\ntrans_y: %s\ntrans_z: %s",
            #     str(trans[0]), str(trans[1]), str(trans[2]))
            user_XR = transR[0]
            user_YR = transR[1]
            user_ZR = transR[2]

            user_XL = transL[0]
            user_YL = transL[1]
            user_ZL = transL[2]

            trans_pointR.x = user_ZR*3
            trans_pointR.y = user_XR*1.5
            trans_pointR.z = -user_YR*2

            trans_pointL.x = user_ZL*3
            trans_pointL.y = user_XL*1.5
            trans_pointL.z = -user_YL*2

            # trans_point.x = 1.0992
            # trans_point.y = -0.4256
            # trans_point.z = 0.6644

            self.mover_right.solver.solve(trans_pointR)
            self.mover_left.solver.solve(trans_pointL)
            self.mover_right.move()
            self.mover_left.move()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


if __name__ == '__main__':
    Teleop()
    rospy.spin()