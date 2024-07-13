# diff_to_ackermann
This package implements a node to convert differential drive commands to ackermann drive commands. It is based on the interface given in the hunter_se package. 

## Use
Once build, the node can be installed as a systemd service so that it is always up: 

` ./install_service.sh`

Once the node is running, launch the hunter_se simulation and command interface using two different terminals:  

- ` ros2 launch hunter_se_gazebo hunter_se_empty_world.launch.py `
- ` ros2 run rqt_robot_steering rqt_robot_steering `

change the topic in the command interface from `/cmd_vel` to `/diffdrive_cmd` to activate the conversion of the commands. 

## Algorithm and Explanations
An Ackermann drive system cannot physically do a pure rotation. In this converter, the pure rotation is transformed in the following maneuvre: 

1. Robot drives straight ahead
2. Robot drives backwards while turning
3. Robot drives straight ahead again (to go back to the starting position)