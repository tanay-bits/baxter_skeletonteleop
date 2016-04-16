#!/usr/bin/env python

import rospy
import numpy as np
from scipy import optimize
import baxter_interface
from baxter_interface import CHECK_VERSION
import tf
from baxter_pykdl import baxter_kinematics
from PyKDL import JntArray
# from tf2_msgs.msg import TFMessage


class Limb_Command:
    def __init__(self,side):
        self.bi_kinem = baxter_kinematics(side)
        self.bi_arm = baxter_interface.limb.Limb(side)
        self.bi_arm.exit_control_mode()
        self.CMD_FREQ = 2
        self.bi_arm.set_command_timeout(2*self.CMD_FREQ)
        rospy.sleep(1)

        self.neutral_pos = np.array([-0.54, -0.67, 0.18, 1.18, -2.5, 0.65, 2.5])
        self.jts_list = self.bi_arm.joint_names()
        self.curr_jts = self.get_curr_joints()
        self.target_jts = self.curr_jts
        self.curr_obj_pos = [2,0,0]
        self.curr_obj_trans = self.calc_obj_trans(self.curr_obj_pos)
        # jts_left = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        
        # self.bi_arm.set_command_timeout(1./self.CMD_FREQ) #[?]
        self.timer_d = rospy.Duration(1./self.CMD_FREQ)
        self.timer = rospy.Timer(self.timer_d,self.callback)

    def set_obj_pos_trans(self,xyz):
        self.curr_obj_pos = xyz
        self.curr_obj_trans = self.calc_obj_trans(xyz)

    def callback(self,event):
        #set current joints, solve desired joints
        #solve joint velocities, set joint velocities

        self.curr_jts = self.get_curr_joints()
        self.target_jts = self.solve_jts(self.curr_obj_pos)
        vel_array = self.solve_vel(self.curr_jts,self.target_jts)
        # print "curr_jts: ", self.curr_jts
        # print "target_jts: ", self.target_jts
        #print "\n===== vel_array: ",vel_array
        self.set_vel_cmd(vel_array)

    def calc_obj_trans(self,xyz):
        #given array of x,y,z coordinates of object, returns matrix transform of object
        return tf.transformations.compose_matrix(translate=(xyz[0],xyz[1],xyz[2]))

    def get_curr_joints(self):
        #return current joint values as array
        jt_dict = self.bi_arm.joint_angles()
        jt_array = np.zeros(len(self.jts_list))
        for i, name in enumerate(self.jts_list):
            jt_array[i] = jt_dict[name]
        return jt_array

    def set_pos_cmd(self,pos_array):
        #given list of joint angles, sets the robot joint values accordingly
        # self.bi_arm.set_command_timeout(1./self.CMD_FREQ) #needs to be done each time?? [?]
        pos_cmd = dict(zip(self.jts_list,pos_array))
        self.bi_arm.set_joint_positions(pos_cmd)
    def set_vel_cmd(self,vel_array):
        #given list of joint angle velocities, sets the robot joint values accordingly
        # self.bi_arm.set_command_timeout(1./self.CMD_FREQ)
        vel_cmd = dict(zip(self.jts_list,vel_array))
        self.bi_arm.set_joint_velocities(vel_cmd)

    def solve_jts(self, xyz):
        #given array of xyz coordinates of obj position, solve for optimal joint values to point at object
        #CONSIDER just solving for the first 6 joints, since wrist rotation doesn't matter for this application
        bounds = [(-1.7 ,1.7),(-2,1),(-3,3),(0,2.6),(-3,3),(-1.5,2),(-3,3)]
        options = {"maxiter":1000,"disp":False} #[?]
        result = optimize.minimize(self.calc_error,self.curr_jts,options=options)
        jts = result.x
        # success = results.success
        return jts
    def solve_vel(self, curr_jts, target_jts):
        #given set of current joint values and target joint values, returns velocity values
        error = target_jts - curr_jts
        # print "Original error",error
        #assign speed according to percentage of error to maximum possible error
        # max_error = np.array([3.4,3,6,2.6,6,3.5,6])
        # error_percent = error / max_error
        # error_percent = np.sqrt(error_percent)
        # error_percent[np.nonzero(error_percent < 0.0001)] = 0 #if error is below a percentage, it's acceptable
        error[np.nonzero(np.abs(error) < 0.001)] = 0
        Kp=0.5 #proportional control constant
        error *= Kp
        error[np.nonzero(error > 1)] = 1
        error[np.nonzero(error < -1)] = -1
        # print "Error : ", error
        max_vel = np.array([2.,2.,2.,2.,4.,4.,4.])
        vel_cmd = max_vel * error

        return vel_cmd

    def solve_fk(self, pos_array):
        #given list of joint angles, calculates transform from base to end-effector
        # jt_array = JntArray(len(pos_array))
        # for i,jt in enumerate(pos_array):
        #     jt_array[i] = jt     #populate JntArray data type
        pos_dict = dict(zip(self.jts_list,pos_array))
        # jt_array = self.bi_kinem.joints_to_kdl('positions',values=pos_dict) #the other way's probably faster, but let's try out this tool...

        pos_rot = self.bi_kinem.forward_position_kinematics(joint_values=pos_dict)
        pos = pos_rot[:3]
        rot = pos_rot[3:]
        euler = tf.transformations.euler_from_quaternion(rot)
        trans=tf.transformations.compose_matrix(angles=euler, translate=pos)
        return trans

    def calc_gripper_to_obj(self, pos_array):
        #given array of joint values, returns position coordinates of object in end-effector frame
        gripper_trans = self.solve_fk(pos_array) #gripper in base frame
        inv_gripper_trans = tf.transformations.inverse_matrix(gripper_trans) #base in gripper frame
        # print "inv_gripper_trans: ",inv_gripper_trans
        # obj_trans = tf.transformations.inverse_matrix(self.curr_obj_trans)
        gripper_to_obj_trans = np.dot(inv_gripper_trans,self.curr_obj_trans)
        gripper_to_obj=tf.transformations.decompose_matrix(gripper_to_obj_trans)
        translation = gripper_to_obj[3]
        return translation

    def calc_error(self, pos_array):
        #given array of joint values, returns error rating of pointing accuracy
        translation = self.calc_gripper_to_obj(pos_array)
        #maintain threshold distance from object
        thresh = 0.5 #half a meter... [?]
        z_error = max(thresh-translation[2],0)**2
        #minimize translation in y and z directions
        pointer_error = translation[0]**2 + translation[1]**2
        #keep joints away from limits, so keep close to neutral joint values
        jt_limit_error = np.sum((pos_array - self.neutral_pos)**2)
        #keep joints close to previous values
        curr_pos_error = np.sum((pos_array - self.curr_jts)**2)
        #assign weights to each error [?]
        w2 = 2 #we really prioritize pointer accuracy
        w1=w3=w4 = 1
        total_error = w1*z_error + w2*pointer_error + w3*jt_limit_error + w4*curr_pos_error
        # print "z_error", z_error
        # print translation
        # print "pointer_error", pointer_error
        # print "jt_limit_error", jt_limit_error
        # print "curr_pos_error", curr_pos_error

        return total_error

    def set_neutral_pos(self):
        neutral_pos_cmd = dict(zip(self.jts_list,self.neutral_pos))
        bi_arm.move_to_joint_positions(neutral_pos_cmd) #blocking command

    def clean_shutdown(self):
        rate = rospy.Rate(10)
        for i in range(10):
            self.bi_arm.exit_control_mode()
            rate.sleep()

def main():
    rospy.init_node("test_velocity_control")
    # bi_arm = baxter_interface.limb.Limb("left")
    # bi_arm.exit_control_mode()
    # bi_arm.set_command_timeout(15)
    # rospy.sleep(1)

    print "Enabling robot"
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    print "Initializing Left Arm"
    bi_left = Limb_Command("left")
    # print "\nJoint angles: ", bi_left.curr_jts
    # print "\nCurrent obj transform: ",bi_left.curr_obj_trans
    # print "\nJoint angles: ", bi_left.curr_jts

    # for idx, name in enumerate(bi_left.bi_kinem._joint_names):
    #     print idx,name


    translation = bi_left.calc_gripper_to_obj(bi_left.curr_jts)
    print "\nObj Coord in End-Eff Frame: ", translation
    # bi_left.callback()
    

    rospy.spin()
    print "Shutting Down"
    bi_left.bi_arm.exit_control_mode()

# def demo():

#     rospy.init_node("test_velocity_control")
#     bi_arm = baxter_interface.limb.Limb("left")
#     bi_arm.exit_control_mode()
#     bi_arm.set_command_timeout(15)
#     rospy.sleep(1)

#     print "Enabling robot"
#     rs = baxter_interface.RobotEnable(CHECK_VERSION)
#     rs.enable()

#     jts_left = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    
#     neutral_pos=np.array([-0.54, -0.67, 0.18, 1.18, -2.5, 0.65, 2.5])
#     neutral_pos_cmd = dict(zip(jts_left,neutral_pos))
#     zero_pos=[0,0,0,0,0,0,0]
#     zero_pos_cmd = dict(zip(jts_left,zero_pos))

#     print "\n ===== Setting joint positions \n"
#     print "NEUTRAL!"
#     # print "Neutral_pos: ", neutral_pos_cmd
#     bi_arm.set_joint_positions(neutral_pos_cmd)
#     # print "Current Joint Angles: ",bi_arm.joint_angles()
#     # print "Current Joint Velocities: ",bi_arm.joint_velocities()
#     rospy.sleep(3)
#     print "ZERO!"
#     bi_arm.set_joint_positions(zero_pos_cmd)
#     rospy.sleep(5)
#     print "\n ===== Setting joint velocities \n"
#     rospy.sleep(2)
#     vel = np.array([0,0,0,0,0,0.5,0])
#     vel_cmd = dict(zip(jts_left,vel))
#     # print "target_vel: ", vel_cmd
#     # bi_arm.set_command_timeout(2)
#     print "FORWARD!"
#     bi_arm.set_joint_velocities(vel_cmd)  #({"left_s0":np.sign(signal_s0)*2,"left_s1":np.sign(signal_s1)}) 

#     vel = -1*vel
#     rospy.sleep(2)
#     vel_cmd = dict(zip(jts_left,vel))
#     # print "target_vel: ", vel_cmd
#     # bi_arm.set_command_timeout(2)
#     print "BACKWARD!"
#     bi_arm.set_joint_velocities(vel_cmd)
#     rospy.sleep(2)

#     print "\n ===== Done! \n"

#     bi_arm.exit_control_mode()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass