#!/usr/bin/env python
import rospy
import tf
import baxter_interface
from geometry_msgs.msg import Point
from skeletonmsgs_nu.msg import Skeletons
from baxter_interface import CHECK_VERSION
from limb_mover import LimbMover

class Teleop:
    def __init__(self):
        rospy.init_node('teleop')
        self.tflistener = tf.TransformListener()
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.rs.state().enabled:
            rospy.logerr("Baxter not enabled")

        self.mover_left = LimbMover("left")
        self.mover_right = LimbMover("right")
        # self.user = 1;
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

            solver_R = self.mover_right.solver
            solver_L = self.mover_left.solver
            
            solver_R.solve(trans_pointR)
            solver_L.solve(trans_pointL)

            # self.mover_right.interface.set_joint_positions(self.mover_right.solver.solution)
            # self.mover_left.move()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

# class Controller:
#     def __init__(self, teleop_obj):


def cb_control(event):
    Kp = 1
    capMaxSpeed = 0.5   #fraction of max speed limit allowed
    deadband = 0.3      #radians
    control_cmdL = dict()
    control_cmdR = dict()

    cur_angsL = teleop_obj.mover_left.interface.joint_angles()
    cur_angsR = teleop_obj.mover_right.interface.joint_angles()
    des_angsL = teleop_obj.mover_left.solver.solution
    des_angsR = teleop_obj.mover_right.solver.solution

    # cur_velsL = teleop_obj.mover_left.interface.joint_velocities()
    # cur_velsR = teleop_obj.mover_right.interface.joint_velocities()
    # des_velsL = 

    for key, val in des_angsL.iteritems():
        eL = val - cur_angsL[key]
        if abs(eL)>deadband:
            if abs(Kp*eL) < max_velsL[key]*capMaxSpeed:
                control_cmdL[key] = Kp*eL
            else:
                if eL<0:
                    control_cmdL[key] = -max_velsL[key]*capMaxSpeed
                else:
                    control_cmdL[key] = max_velsL[key]*capMaxSpeed
        else:
            control_cmdL[key] = 0

    for key, val in des_angsR.iteritems():
        eR = val - cur_angsR[key]
        if abs(eR)>deadband:
            if abs(Kp*eR) < max_velsR[key]*capMaxSpeed:
                control_cmdR[key] = Kp*eR
            else:
                if eR<0:
                    control_cmdR[key] = -max_velsR[key]*capMaxSpeed
                else:
                    control_cmdR[key] = max_velsR[key]*capMaxSpeed
        else:
            control_cmdL[key] = 0

    teleop_obj.mover_left.move(control_cmdL)
    # teleop_obj.mover_right.move(control_cmdR)
        

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


if __name__ == '__main__':
    CONTROL_FREQ = 100   #Hz
    teleop_obj = Teleop()
    ang_limsL, max_velsL = joint_lims('left')
    ang_limsR, max_velsR = joint_lims('right')
   
    while teleop_obj.rs.state().enabled and not rospy.is_shutdown():
        dt = rospy.Duration(1./CONTROL_FREQ)
        rospy.Timer(dt, cb_control)    
    
    rospy.spin()