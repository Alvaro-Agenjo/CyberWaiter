# CyberWaiter

To use the alternative KDL method is necesary to run the following node: ros2 run cyberwaiter_control kin_solver 

The node is prepared to accept a geometry_msgs/Pose through the topic /gui_bridge/cmd as in the following example:

ros2 topic pub -1 /goals/goal_coord geometry_msgs/msg/Pose "{position: {x: 0.1084636463212604, y: 0.2688863053296338, z: 0.50060022241172}, orientation: {x: -0.7122172185325976, y: -0.1552003926532003, z: -0.48831461977670154, w: 0.4798002749669054}}"



For testing pruposes, is recomended to obtain the Pose command from topic /tcp_pose_broadcaster/pose

