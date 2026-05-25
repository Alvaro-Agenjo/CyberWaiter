# CyberWaiter

To use the alternative KDL method is necesary to run the following node: ros2 run cyberwaiter_control kin_solver 

The node is prepared to accept a geometry_msgs/Pose through the topic /gui_bridge/cmd as in the following example:

ros2 topic pub -1 /goals/goal_coord geometry_msgs/msg/Pose "{position: {x: -0.12360693427025358, y: 0.4594488816440156, z: 0.1527464906094069}, orientation: {x: -0.7034602005375664, y: -0.12721179398320612, z: -0.1292500879408832, w: 0.6872083530476731}}"



For testing pruposes, is recomended to obtain the Pose command from topic /tcp_pose_broadcaster/pose

