#!/usr/bin/env python
###########
# IMPORTS #
###########
import rospy
import tf
import baxter_interface
from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray
from skeletonmsgs_nu.msg import Skeletons
from baxter_core_msgs.msg import EndEffectorState
from baxter_interface import CHECK_VERSION
from new_limb_mover import NewLimbMover
import numpy as np

####################
# GLOBAL VARIABLES #
####################
FREQ_DIV = 30   #frequency divider for checking "key" skeleton
ANG_MULT = 20
DIST_MULT = 1
CONTROL_FREQ = 100  #Hz
KP = 10
CAPMAXSPEED = 0.3   #fraction of max speed limit allowed
DEADBAND = 0.3      #radians
RUMBLE_DURATION = 1

class NewTeleop:
    def __init__(self):
        rospy.init_node('new_teleop')
        wiiFlag = rospy.get_param('~using_wii')
        self.tflistener = tf.TransformListener()
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.rs.state().enabled:
            rospy.logerr("Baxter not enabled...Enabling now")
            self.rs.enable()

        if wiiFlag:
        # Define publisher for rumble:
            self.pub = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray, queue_size=1)
            rospy.sleep(2)
            self.rumFlag = True

        self.mover_left = NewLimbMover("left")
        self.mover_right = NewLimbMover("right")

        if wiiFlag:
            self.gripperL = baxter_interface.Gripper("left")
            # self.gripperR = baxter_interface.Gripper("right")
            self.gripperL.calibrate()
            # self.gripperR.calibrate()

        self.ang_limsL, self.max_velsL = joint_lims('left')
        self.ang_limsR, self.max_velsR = joint_lims('right')
        
        self.start_flagL = False
        self.start_flagR = False
        self.count = 0
        self.key_index = 0
        self.key_id = 1

        # Define two subscribers (one for each arm) to listen to /skeletons:
        rospy.Subscriber("/skeletons", Skeletons, self.cb_skelL, queue_size=1)
        rospy.Subscriber("/skeletons", Skeletons, self.cb_skelR, queue_size=1)
        
        if wiiFlag:
        # Define a subscriber to listen to /joy for gripper control:
            rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)

        # Define a timer to run velocity control:
        dt = rospy.Duration(1./CONTROL_FREQ)
        self.timer = rospy.Timer(dt, self.cb_control)
        
        if wiiFlag:
        # Define a timer to check if Wiimote needs to rumble:
            check_for_rum_every = rospy.Duration(0.1)
            self.rum_timer = rospy.Timer(check_for_rum_every, self.cb_rum)

        rospy.on_shutdown(self._cleanup)

    def cb_skelL(self, message):
        # get key user:
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

        # get target end-effector position for Baxter and solve IK:
        if (self.tflistener.frameExists('/torso_'+str(user)) and 
            self.tflistener.frameExists('/left_hand_'+str(user))):
            try:
                (transL, rotL) = self.tflistener.lookupTransform('/torso_' + str(user),
                    '/left_hand_' + str(user), rospy.Time(0))

                user_XL = transL[0]
                user_YL = transL[1]
                user_ZL = transL[2]

                if not self.start_flagL:
                    pmins, pmaxs = start_box(0.10, -0.05, 0.3, 0.6)
                    if (pmins[0]<user_XL<pmaxs[0] and pmins[1]<user_YL<pmaxs[1] and pmins[2]<user_ZL<pmaxs[2]):
                        self.start_flagL = True

                if (self.start_flagL and self.start_flagR):
                    target_transL = np.array([user_ZL*3, user_XL*1.5, -user_YL*2])                
                    if self.mover_left.iksolve(target_transL):
                        rospy.loginfo("Solution Found - LEFT")
                    else:
                        rospy.logwarn("No Solution Found - LEFT")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass        
        
        return

    def cb_skelR(self, message):
        # get key user:
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

        # get target end-effector position for Baxter and solve IK:
        if (self.tflistener.frameExists('/torso_'+str(user)) and 
            self.tflistener.frameExists('/right_hand_'+str(user))):
            try:
                (transR, rotR) = self.tflistener.lookupTransform('/torso_' + str(user),
                    '/right_hand_' + str(user), rospy.Time(0))

                user_XR = transR[0]
                user_YR = transR[1]
                user_ZR = transR[2]

                if not self.start_flagR:
                    pmins, pmaxs = start_box(0.10, -0.05, 0.3, 0.6)
                    if (pmins[0]<user_XR<pmaxs[0] and pmins[1]<user_YR<pmaxs[1] and pmins[2]<user_ZR<pmaxs[2]):
                        self.start_flagR = True

                if (self.start_flagL and self.start_flagR):
                    target_transR = np.array([user_ZR*3, user_XR*1.5, -user_YR*2]) 
                    if self.mover_right.iksolve(target_transR):
                        rospy.loginfo("Solution Found - RIGHT")
                    else:
                        rospy.logwarn("No Solution Found - RIGHT")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass        
        
        return

    def cb_control(self, event):
        control_cmdL = dict()
        control_cmdR = dict()

        if (self.start_flagL and self.start_flagR):
            cur_angsL = self.mover_left.interface.joint_angles()
            cur_angsR = self.mover_right.interface.joint_angles()
            des_angsL = self.mover_left.target_jts_dict
            des_angsR = self.mover_right.target_jts_dict

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
            ang = np.arccos(abs(v2[1])/np.linalg.norm(v2))
            dist = v2[1]
            cost = ANG_MULT*ang + DIST_MULT*dist
            data.append([i, s.userid, cost])
        val, idx = min((val[2], idx) for (idx, val) in enumerate(data))
        self.key_index = data[idx][0]
        self.key_id = data[idx][1]
        return

    def cb_joy(self, message):
        if message.buttons[3] == 1:
            self.gripperL.close()

        if message.buttons[2] == 1:
            self.gripperL.open()        
        return

    def cb_rum(self, event):      
        if self.gripperL._state.force > 20 and self.rumFlag == True:
            rum = JoyFeedback()
            rum.type = JoyFeedback.TYPE_RUMBLE
            rum.id = 0
            rum.intensity = 0.51
            msg = JoyFeedbackArray()
            msg.array = [rum]            
            self.pub.publish(msg)
            rospy.sleep(0.3)
            rum.intensity = 0.0
            msg.array = [rum]
            self.pub.publish(msg)
            self.rumFlag = False

        if self.gripperL._state.force == 0:
            self.rumFlag = True

    def _cleanup(self):
        rospy.loginfo("Shutting down...")
        self.mover_left.interface.exit_control_mode()
        self.mover_right.interface.exit_control_mode()


# auxiliary functions:
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


# main function:
if __name__ == '__main__':
    NewTeleop()
    rospy.spin()