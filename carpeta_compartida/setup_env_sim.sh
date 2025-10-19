source ~/carpeta_compartida/gallium/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

unset ROS_IP

alias launch_tiago_simulation="roslaunch tiago_moveit_config demo.launch end_effector:=pal-gripper"