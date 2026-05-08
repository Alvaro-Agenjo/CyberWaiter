# CyberWaiter

To use the alternative KDL method is necesary to run the following node: ros2 run cyberwaiter_control kin_solver 

The node is prepared to accept a geometry_msgs/Pose through the topic /gui_bridge/cmd as in the following example:

ros2 topic pub -1 /gui_bridge/cmd geometry_msgs/msg/Pose "{position: {x: -0.0, y: -0.0, z: 0.4}, orientation: {x: 0.013014382549689035, y: 0.0042876706606361555, z: -0.7171280118555123, w: 0.6968067568123292}}"


For testing pruposes, is recomended to obtain the Pose command from topic /tcp_pose_broadcaster/pose

