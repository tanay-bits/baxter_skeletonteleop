Skeleton Tracking and IK Based Teleoperation of the Baxter Robot
==============
This ROS package allows convenient remote teleoperation of the [Baxter](http://www.rethinkrobotics.com/baxter/) robot using skeleton tracking information based on the NITE skeleton tracking library.

[![demo](http://i.imgur.com/mfcdN3q.png)](https://vimeo.com/159668224 "Click to watch (old) demo!")

The following packages are pre-requisites, in addition to the [Baxter SDK](https://github.com/RethinkRobotics/baxter):

+   [skeletontracker_nu](https://github.com/NxRLab/skeletontracker_nu)
+   [skeletonmsgs_nu](https://github.com/NxRLab/skeletonmsgs_nu)
+   [baxter_pykdl](https://github.com/RethinkRobotics/baxter_pykdl)
+   Optionally, if you want control over gripper and haptic (vibration) feedback, [wiimote](http://wiki.ros.org/wiimote)

The following answers are great resources for setting up skeleton tracking:

+   [how-can-i-setup-skeleton-tracking-using-a-kinect-and-ros-indigo-on-ubuntu](http://answers.ros.org/question/214421/how-can-i-setup-skeleton-tracking-using-a-kinect-and-ros-indigo-on-ubuntu-1404/#220498)
+   [asus-xtion-problems-with-ubuntu-1204-running-ros-fuerte](http://answers.ros.org/question/109411/asus-xtion-problems-with-ubuntu-1204-running-ros-fuerte/#109831)

To pair your Wiimote with your computer, follow this [tutorial](http://wiki.ros.org/wiimote/Tutorials/StartingWiimoteNode).

After enabling Baxter, simply roslaunch `baxter_skeletonteleop.launch` to start the program:
`roslaunch baxter_skeletonteleop baxter_skeletonteleop.launch`

Optionally, if you have your Wiimote paired, use the followinng launch command:
`roslaunch baxter_skeletonteleop baxter_skeletonteleop.launch wiimote:=true`

And, if you want to use my (now demoted) old approach which relied solely on Baxter's IK Service, type:
`roslaunch baxter_skeletonteleop baxter_skeletonteleop.launch old_method:=true`

The default launch uses `new_teleop.py` and `new_limb_mover.py`, whereas the old method uses `teleop.py`, `limb_mover.py` and `ik_solver.py`. You should soon see an RViz window with the depth video overlaid on the skeleton tracker. Stand in front of your OpenNI-compliant depth-sensor (such as Kinect or Asus Xtion), and bring your hands to the start position (both hands near the torso) to begin tele-operating the robot. If there are multiple people coming in and out of the depth sensor's field of view, Baxter will track whichever user is most central.

If you're holding your paired Wiimote, press the B button to close Baxter's gripper (currently only one arm supported for gripping) and the A button to open it. If Baxter grabs an object, the Wiimote would momentarily rumble for user feedback.

The basic flow of processes is as follows:
**skeleton tracking --> mapping of human's hand position to robot's end-effector position --> custom iterative numerical inverse kinematics --> proportional joint-space velocity control**

The robot's joint speeds are intentionally capped to 40% of maximum, as a safety precaution. The current scaling factors for mapping human hand position to Baxter's end-effector position work pretty well for most people in my experiments, but one may need to tweak them if that isn't the case. This will also be necessary if your depth sensor's coordinate axes are different from mine (I'm using an Asus Xtion Pro Live). The scaling factors can be estimated by noting Baxter’s endpoint state at the outward limits of its reach in positive and negative X,Y,Z directions, perhaps using `rostopic echo /robot/limb/<left or right>/endpoint_state -c` on your terminal, and comparing that with the human user's arm measurements. The relevant lines of code to change these factors are: 

+   [Left arm scaling factors](https://github.com/tanay-bits/baxter_skeletonteleop/blob/75fbf847f58b4aba93434ec0bdde64d8c1c6ab44/scripts/teleop.py#L106-L108)
+   [Right arm scaling factors](https://github.com/tanay-bits/baxter_skeletonteleop/blob/75fbf847f58b4aba93434ec0bdde64d8c1c6ab44/scripts/teleop.py#L156-L158)

The inverse kinematics routine implemented here runs multivariate optimization of a squared error objective function over the domain of forward kinematics solutions (joint configurations). Only the first 6 joint angles are considered, since the last joint angle (wrist roll) doesn't really matter here. The joint angle 6-vectors are bounded by their physical limits. The objective function consists of two costs:

+	Square of the distance between the end-effector position arrived at via forward kinematics, and the target end-effector position determined from skeleton tracking. This cost is most important, so has higher weight multiplied to it.
+	Square of the L2 norm of difference between the current actual joint configuration and the joint configuration being evaluated by the optimizer (the guess). This is of secondary importance, so has lower weight.

The two weights are global variables that can be changed in the beginning of `new_limb_mover.py`. Additionally, to aid the optimizer, I provide it the Jacobian of the above objective function.