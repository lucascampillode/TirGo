source ~/carpeta_compartida/gallium/setup.bash

export ROS_MASTER_URI=http://tiago-222c:11311
export ROS_IP=10.68.0.130

# export ROS_MASTER_URI=http://localhost:11311
# unset ROS_IP

alias launch_tiago_simulation="roslaunch tiago_moveit_config demo.launch end_effector:=pal-gripper"


# source ~/carpeta_compartida/gallium/setup.bash

# export ROS_MASTER_URI=http://10.172.209.200:11311
# export ROS_IP=10.172.209.2