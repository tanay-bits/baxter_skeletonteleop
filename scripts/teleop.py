#!/usr/bin/env python
###############
# ROS IMPORTS  
###############
import rospy
import tf
import baxter_interface
from geometry_msgs.msg import Point
from skeletonmsgs_nu.msg import Skeletons
from baxter_interface import CHECK_VERSION
from limb_mover import LimbMover

###################
# NON-ROS IMPORTS #
###################
import numpy as np

####################
# GLOBAL VARIABLES #
####################
FREQ_DIV = 30   #frequency divider for checking "key" skeleton
ANG_MULT = 20
DIST_MULT = 1
CONTROL_FREQ = 100   #Hz
KP = 1
CAPMAXSPEED = 0.3   #fraction of max speed limit allowed
DEADBAND = 0.3      #radians

class Teleop:
    def __init__(self):
        rospy.init_node('teleop')
        self.tflistener = tf.TransformListener()
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.rs.state().enabled:
            rospy.logerr("Baxter not enabled")

        self.mover_left = LimbMover("left")
        self.mover_right = LimbMover("right")
        
        # define a subscriber to listen to /skeletons:
        self.start_flag = False
        self.count = 0
        self.key_index = 0
        self.key_id = 1
        rospy.Subscriber("/skeletons", Skeletons, self.cb_skel, queue_size=1)

        self.ang_limsL, self.max_velsL = joint_lims('left')
        self.ang_limsR, self.max_velsR = joint_lims('right')

        dt = rospy.Duration(1./CONTROL_FREQ)
        self.timer = rospy.Timer(dt, self.cb_control)
        
    def cb_skel(self, message):
        if len(message.skeletons) == 0:
            return
        if self.count%FREQ_DIV == 0:
            self.get_key_user(message.skeletons)
        self.count += 1
        if self.key_index < len(message.skeletons) and \
                message.skeletons[self.key_index].userid == self.key_id:
            skel = message.skeletons[self.key_index]
        else:
            for i,skel in enumerate(message.skeletons):
                if skel.userid == self.key_id:
                    found = True
                    break
                found = False
            if not found:
                rospy.logwarn("Could not find a skeleton userid that matches the key user")
                return
        
        user = skel.userid

        # target points for Baxter:
        trans_pointL = Point()
        trans_pointR = Point()

        try:
            (transR, rotR) = self.tflistener.lookupTransform('/torso_' + str(user),
                '/right_hand_' + str(user), rospy.Time(0))
            (transL, rotL) = self.tflistener.lookupTransform('/torso_' + str(user),
                '/left_hand_' + str(user), rospy.Time(0))
            user_XR = transR[0]
            user_YR = transR[1]
            user_ZR = transR[2]
            user_XL = transL[0]
            user_YL = transL[1]
            user_ZL = transL[2]

            if not self.start_flag:
                pmins, pmaxs = start_box(0.10, -0.05, 0.3, 0.60)
                if (pmins[0]<user_XL<pmaxs[0] and pmins[1]<user_YL<pmaxs[1] and pmins[2]<user_ZL<pmaxs[2] and 
                    pmins[0]<user_XR<pmaxs[0] and pmins[1]<user_YR<pmaxs[1] and pmins[2]<user_ZR<pmaxs[2]):
                    self.start_flag = True

            else:
                trans_pointR.x = user_ZR*3
                trans_pointR.y = user_XR*1.5
                trans_pointR.z = -user_YR*2

                trans_pointL.x = user_ZL*3
                trans_pointL.y = user_XL*1.5
                trans_pointL.z = -user_YL*2
                
                self.mover_right.solver.solve(trans_pointR)
                self.mover_left.solver.solve(trans_pointL)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return

    def cb_control(self, event):
        control_cmdL = dict()
        control_cmdR = dict()

        if self.start_flag:
            cur_angsL = self.mover_left.interface.joint_angles()
            cur_angsR = self.mover_right.interface.joint_angles()
            des_angsL = self.mover_left.solver.solution
            des_angsR = self.mover_right.solver.solution

            for key, val in des_angsL.iteritems():
                eL = val - cur_angsL[key]
                if abs(eL)>DEADBAND:
                    if abs(KP*eL) < self.max_velsL[key]*CAPMAXSPEED:
                        control_cmdL[key] = KP*eL
                    else:
                        if eL<0:
                            control_cmdL[key] = -self.max_velsL[key]*CAPMAXSPEED
                        else:
                            control_cmdL[key] = self.max_velsL[key]*CAPMAXSPEED
                else:
                    control_cmdL[key] = 0

            for key, val in des_angsR.iteritems():
                eR = val - cur_angsR[key]
                if abs(eR)>DEADBAND:
                    if abs(KP*eR) < self.max_velsR[key]*CAPMAXSPEED:
                        control_cmdR[key] = KP*eR
                    else:
                        if eR<0:
                            control_cmdR[key] = -self.max_velsR[key]*CAPMAXSPEED
                        else:
                            control_cmdR[key] = self.max_velsR[key]*CAPMAXSPEED
                else:
                    control_cmdR[key] = 0

            self.mover_left.move(control_cmdL)
            self.mover_right.move(control_cmdR)     

    def get_key_user(self, skels):
        data = []
        for i,s in enumerate(skels):
            v2 = np.array([s.head.transform.translation.x,
                           s.head.transform.translation.z])
            ang = np.arccos(v2[1]/np.linalg.norm(v2))
            dist = v2[1]
            cost = ANG_MULT*ang + DIST_MULT*dist
            data.append([i, s.userid, cost])
        val, idx = min((val[2], idx) for (idx, val) in enumerate(data))
        self.key_index = data[idx][0]
        self.key_id = data[idx][1]
        return


#auxiliary functions:
def start_box(xcenter, ycenter, zcenter, boxlength):
    to_add = boxlength/2.
    xmin = xcenter - to_add
    xmax = xcenter + to_add
    ymin = ycenter - to_add
    ymax = ycenter + to_add
    zmin = zcenter - to_add
    zmax = zcenter + to_add

    pmins = [xmin, ymin, zmin]
    pmaxs = [xmax, ymax, zmax]
    return pmins, pmaxs

def joint_lims(limb):
    ang_lims = dict([(limb+'_s0',(-1.7, 1.7)),
        (limb+'_s1',(-2.14, 1.04)),
        (limb+'_e0',(-3.05, 3.05)),
        (limb+'_e1',(-0.05, 2.61)),
        (limb+'_w0',(-3.05, 3.05)),
        (limb+'_w1',(-1.57, 2.09)),
        (limb+'_w2',(-3.05, 3.05))])
    max_vels = dict([(limb+'_s0',2),
        (limb+'_s1',2),
        (limb+'_e0',2),
        (limb+'_e1',2),
        (limb+'_w0',4),
        (limb+'_w1',4),
        (limb+'_w2',4)])
    return ang_lims, max_vels


#main function:
if __name__ == '__main__':
    Teleop()
    rospy.spin()