#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import numpy as np
from sensor_msgs.msg import JointState
import baxter_interface

class IKsolver_gripper:
    def __init__(self, limb):
        self.limb = limb    #just a string
        self.interface = baxter_interface.Limb(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns)
        self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        self.solution = dict()
        # self.solution_found = False
        self.qtarget = [
            # Human-like mapping (front of camera  = front of the wrist)
            Quaternion(
                x=0,
                y=0.7071067811865475244, # sqrt(0.5)
                z=0,
                w=0.7071067811865475244  # sqrt(0.5)
            # Camera is pointing down
            ), Quaternion(
                x=0,
                y=1,
                z=0,
                w=0
            ),
        ]

    def solve(self, ptarget, target_rot): #ptarget should be a Point(); target_rot 'FRONT' or 'DOWN'
        # self.solution_found = False
        ikreq = SolvePositionIKRequest() #service request object
        hdr = Header(
            stamp=rospy.Time.now(), frame_id='base')
        if target_rot == 'FRONT':
            pose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=ptarget,
                    orientation=self.qtarget[0]
                ),
            )
        elif target_rot == 'DOWN':
            pose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=ptarget,
                    orientation=self.qtarget[1]
                ),
            )

        ikreq.pose_stamp.append(pose)

        try:
            resp = self.iksvc(ikreq)           
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        if resp.isValid[0]==True:
            self.solution = dict(zip(resp.joints[0].name,
                resp.joints[0].position))
            # self.solution_found = True
            rospy.loginfo("Exact Solution Found, %s" % self.limb, self.solution)
            return True
      
        else:
            # if target_rot == 'FRONT':
            #     num_soft_targets = 4
            #     for i in xrange(num_soft_targets):
            #         noise = np.random.normal(0,0.1,3)
            #         ptarget_soft = Point(x=ptarget.x+noise[0],
            #             y=ptarget.y+noise[1],
            #             z=ptarget.z+noise[2])
            #         pose_soft = PoseStamped(
            #             header=hdr,
            #             pose=Pose(
            #                 position=ptarget_soft,
            #                 orientation=self.qtarget[0]
            #             ),
            #         )
            #         ikreq.pose_stamp.append(pose_soft)
            
            # elif target_rot == 'DOWN':
            #     num_soft_targets = 4
            #     for i in xrange(num_soft_targets):
            #         noise = np.random.normal(0,0.1,3)
            #         ptarget_soft = Point(x=ptarget.x+noise[0],
            #             y=ptarget.y+noise[1],
            #             z=ptarget.z+noise[2])
            #         pose_soft = PoseStamped(
            #             header=hdr,
            #             pose=Pose(
            #                 position=ptarget_soft,
            #                 orientation=self.qtarget[1]
            #             ),
            #         )
            #         ikreq.pose_stamp.append(pose_soft)

            for counter in xrange(50):
                try:
                    resp = self.iksvc(ikreq)           
                except (rospy.ServiceException, rospy.ROSException), e:
                    rospy.logerr("Service call failed: %s" % (e,))
                    return False

                if any(resp.isValid):
                    for i in range(len(resp.isValid)):
                        if resp.isValid[i]==True:
                            self.solution = dict(zip(resp.joints[i].name,
                                resp.joints[i].position))
                            # self.solution_found = True
                            rospy.loginfo("Exact Solution Found, %s" % self.limb, self.solution)
                            return True
                else:
                    rospy.logwarn("INVALID POSE for %s" % self.limb)
                    jointlims = dict([('s0',(-1.7, 1.7)),
                        ('s1',(-2.14, 1.04)),
                        ('e0',(-3.05, 3.05)),
                        ('e1',(-0.05, 2.61)),
                        ('w0',(-3.05, 3.05)),
                        ('w1',(-1.57, 2.09)),
                        ('w2',(-3.05, 3.05))])
                    
                    noiselist = []
                    for key, val in jointlims.iteritems():
                        noise = np.random.uniform(val[0], val[1])
                        noiselist.append(noise)

                    js = JointState()
                    js.header = hdr
                    i = 0
                    for key, val in self.interface.joint_angles().iteritems():
                        js.name.append(key)
                        js.position.append(val+noiselist[i])
                        i += 1

                    # ikreq.seed_angles = [js]*(num_soft_targets+1)
                    ikreq.seed_angles = [js]
