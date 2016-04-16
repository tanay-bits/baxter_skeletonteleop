#!/usr/bin/env python
import rospy
import baxter_interface
from baxter_pykdl import baxter_kinematics
import numpy as np
from scipy import optimize

class NewLimbMover:
    def __init__(self, limb):
        self.limb = limb    #just a string
        self.interface = baxter_interface.Limb(limb)
        self.kinem = baxter_kinematics(limb)
        self.jt_names = self.interface.joint_names()
        self.curr_jts = self.get_curr_joints()
        self.target_jts = self.curr_jts
        self.target_jts_dict = self.interface.joint_angles()

    def get_curr_joints(self):
        '''Return current joint angles as ndarray.'''

        jt_dict = self.interface.joint_angles()
        jt_array = np.zeros(len(self.jt_names))
        for i, name in enumerate(self.jt_names):
            jt_array[i] = jt_dict[name]
        return jt_array

    def iksolve(self, ptarget): 
        '''Given ndarray of target end-effector position [x,y,z] in Baxter's base frame,
        update ndarray as well as dict of target joint angles.'''

        self.curr_jts = self.get_curr_joints()
        self.target_jts = self.find_best_jts(ptarget)
        self.target_jts_dict = dict(zip(self.jt_names, self.target_jts))
        return True

    def find_best_jts(self, ptarget):
        '''Given ndarray of target end-effector position [x,y,z], solve for optimal joint values (first 6 joints)
        to point at object. Returns ndarray of all joint angles (first 6 optimal, last=0).'''

        jt_bounds = [(-1.7 ,1.7),(-2.14, 1.04),(-3.05, 3.05),(-0.05, 2.61),(-3.05, 3.05),(-1.57, 2.09)]
        # jt_bounds = np.array([(-1.7 ,1.7),(-2.14, 1.04),(-3.05, 3.05),(-0.05, 2.61),(-3.05, 3.05),(-1.57, 2.09)])
        solver_options = {"maxiter":1000,"disp":False}
        result = optimize.minimize(self.calc_error, self.curr_jts[:-1], args=(ptarget),
            options=solver_options, bounds=jt_bounds)
        jts = result.x
        jts = np.hstack((jts, 0))
        # success = results.success
        return jts

    def calc_error(self, jt_values, ptarget):
        '''Given ndarray of first 6 joint angles, and target end-effector position,
        returns a weighted error objective function.'''

        fk_translation = self.fksolver_trans(np.hstack((jt_values,0)))
        error_gripper_position = (np.linalg.norm(ptarget - fk_translation))**2
        error_jts_travel = (np.linalg.norm(jt_values - self.curr_jts[:-1]))**2

        WEIGHT_GRIPPER_POSITION = 2 
        WEIGHT_JTS_TRAVEL = 1
        
        total_error = WEIGHT_GRIPPER_POSITION*error_gripper_position + WEIGHT_JTS_TRAVEL*error_jts_travel
        return total_error

    def fksolver_trans(self, jt_angles):
        '''Given ndarray of all 7 joint angles, calculates translation from base to end-effector'''
        jt_dict = dict(zip(self.jt_names, jt_angles))
        trans_rot = self.kinem.forward_position_kinematics(joint_values=jt_dict)
        trans = trans_rot[:3]
        return trans

    def move(self, des_joint_vels):
        self.interface.set_joint_velocities(des_joint_vels)