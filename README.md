# CyberWaiter

To use the alternative KDL method is necesary to run the following nodes: 
- ros2 run cyberwaiter_control kin_solver 
- ros2 run cyberwaiter_control gripper
- ros2 run cyberwaiter_control navigator


# navigator

Presenta distintos tipos de trayectoria para el movimiento del efector fina. Recibe un mensage propio de tipo Movement. Este mensaje contiene un campo point donde se indica la posicion final del movimiento y un campo modo donde se especifica el tipo de trayectoria.
	- TO_POINT: se mueve al punto indicado sin puntos de paso.
	- APROX: se mueve al punto final utilizando un punto de paso situado 10 cm por delante del punto punial.
	-DESPLAZAMIENTO: realiza un movimiento de U invertida elevando el TCP 17 cm por encima de la ubicacion original y final.
	

# Movement	
The node is prepared to accept a cyberwaiter_msgs/Movement through the topic /movimiento as in the following examples:

## - TO_POINT:
	ros2 topic pub -1 /movimiento cyberwaiter_msgs/msg/Movement "{point: {position: {x: -0.2789991593758533, y: -0.27384208217651353, z: 0.17388577056763277}, orientation: {x: -0.2699543169297954, y: 0.6680290140952475, z: -0.6933453603863909, w: 0.01158077409177816}}, modo: {data: TO_POINT}, gripper_close: 1}"
## - APROX: 
	ros2 topic pub -1 /movimiento cyberwaiter_msgs/msg/Movement "{point: {position: {x: -0.1789991593758533, y: -0.47384208217651353, z: 0.12388577056763277}, orientation: {x: -0.3699543169297954, y: 0.6680290140952475, z: -0.6933453603863909, w: 0.01158077409177816}}, modo: {data: APROX}, gripper_close: 0}"

## - DESPLAZAMIENTO:	
	ros2 topic pub -1 /movimiento cyberwaiter_msgs/msg/Movement "{point: {position: {x: 0.0789991593758533, y: -0.57384208217651353, z: 0.08388577056763277}, orientation: {x: -0.3699543169297954, y: 0.6680290140952475, z: -0.6933453603863909, w: 0.01158077409177816}}, modo: {data: DESPLAZAMIENTO}, gripper_close: 1-}"


# gripper

the third field named gripper_close is used to indicate the final statte of the gripper.
 1: close
 0: no action 
 -1: open

For testing pruposes, is recomended to obtain the Pose command from topic /tcp_pose_broadcaster/pose

