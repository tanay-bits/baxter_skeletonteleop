#!/usr/bin/env python
import rospy
import baxter_interface
from baxter_skeletonteleop.ik_solver import IKsolver

class LimbMover:
    def __init__(self,limb):
        self.limb = limb
        self.interface = baxter_interface.Limb(limb)
        self.interface.set_joint_position_speed(0.5)
        self.solver = IKsolver(limb)

    def move(self):
        self.interface.move_to_joint_positions(self.solver.solution)