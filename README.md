Skeleton Tracking and IK Based Teleoperation of the Baxter Robot
==============
This ROS package allows convenient remote teleoperation of the [Baxter](http://www.rethinkrobotics.com/baxter/) robot using skeleton tracking information based on the NITE skeleton tracking library. The following packages are pre-requisites, in addition to the [Baxter SDK](https://github.com/RethinkRobotics/baxter):

+   [skeletontracker_nu](https://github.com/NxRLab/skeletontracker_nu)
+   [skeletonmsgs_nu](https://github.com/NxRLab/skeletonmsgs_nu)

After enabling Baxter, simply roslaunch *baxter_skeletonteleop.launch* to start the program. You should soon see an RViz window with the depth video overlaid on the skeleton tracker. Stand in front of your OpenNI-compliant depth-sensor (such as Kinect or Asus Xtion), and bring your hands to the start position (both hands near the torso) to begin tele-operating the robot. If there are multiple people coming in and out of the depth sensor's field of view, Baxter will track whichever user is most central.

The basic flow of processes is as follows:
**skeleton tracking --> scaled mapping of human's hand position to robot's end-effector position --> Iterative numerical inverse kinematics --> proportional joint-space velocity control**

The robot's joint speeds are intentionally capped to 30% of maximum, as a safety precaution. The current scaling factors for mapping human hand position to Baxter's end-effector position work pretty well for most people in my experiments, but one may need to tweak them if that isn't the case. This will also be necessary if your depth sensor's coordinate axes are different from mine (I'm using an Asus Xtion Pro Live).

[![demo](http://i.imgur.com/mfcdN3q.png)](https://vimeo.com/159668224 "Click to watch demo!")