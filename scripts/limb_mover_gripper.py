#!/usr/bin/env python
import rospy
import baxter_interface
from ik_solver_gripper import IKsolver_gripper

class LimbMover_gripper:
    def __init__(self, limb):
        self.limb = limb
        self.interface = baxter_interface.Limb(limb)
        # self.interface.set_joint_position_speed(0.3)
        self.solver = IKsolver_gripper(limb)

    def move(self, des_joint_vels):
        # self.interface.set_joint_positions(self.solver.solution)
        self.interface.set_joint_velocities(des_joint_vels)