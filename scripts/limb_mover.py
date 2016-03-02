#!/usr/bin/env python
import rospy
import baxter_interface
from ik_solver import IKsolver

class LimbMover:
    def __init__(self, limb):
        self.limb = limb
        self.interface = baxter_interface.Limb(limb)
        self.interface.set_joint_position_speed(0.3)
        self.solver = IKsolver(limb)
        # self.last_move_time = rospy.Time.now()

    def move(self, des_joint_vels):
        # self.interface.set_joint_positions(self.solver.solution)
        self.interface.set_joint_velocities(des_joint_vels)