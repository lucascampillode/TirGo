#! /bin/bash
#
# Check if the $HOME/.pal/<robot>_2dnav  folder is available and creates it if needed before
# launching move_base with config file from $HOME/.pal .
#

# Check parameters:
if [ $# -lt 1 ]; then
  echo "Usage: $0 <robot> <rgbd_sensors> <subtype> <laser_model> <robot_specific_parameters>"
  exit 1
else
  robot=$1
fi

# Figure out from where to load the navigation configuration
# Hierarchy is DEPLOYED_WS > .PAL > /OPT
config_base_path=`rospack find pal_navigation_cfg_${robot}`
rc=$?
if [ $rc != 0 ]; then 
  exit $rc
fi

ws_loc="$(echo $config_base_path | grep $HOME)"
if [ ! -z "$ws_loc" ]; then
  echo "Found deployed navigation configuration"
else
  custom_config_path=$HOME/.pal/${robot}_2dnav
  if [ -d $custom_config_path ]; then
    echo "Found custom navigation configuration"
    config_base_path=$custom_config_path
  else
    if [ -d $config_base_path ]; then
      echo "Found default navigation configuration"
    else
      echo "Error, no navigation configuration found in the robot."
      exit 1
    fi
  fi
fi

echo "Using the navigation configuration from ${config_base_path}."

# If number of parameters is less than two, it has no RGBD sensors
if [ $# -lt 2 ]; then
    RGBD_SENSORS="false"
else
    RGBD_SENSORS="$2"
fi

# If number of parameters is less than three, it has default robot subtype
if [ $# -lt 3 ]; then
    SUBTYPE="custom"
else
    SUBTYPE="$3"
fi

# If number of parameters is less than three, it has default robot laser_model
if [ $# -lt 4 ]; then
    LASER_MODEL="false"
else
    LASER_MODEL="$4"
fi

# pass directly the last parameters
if [ $# -lt 5 ]; then
    ROBOT_SPECIFIC_PARAMS=""
else
    ROBOT_SPECIFIC_PARAMS=${@:5}
fi

# Run move_base
roslaunch ${robot}_2dnav move_base.launch config_base_path:=$config_base_path rgbd_sensors:=$RGBD_SENSORS subtype:=$SUBTYPE laser_model:=$LASER_MODEL $ROBOT_SPECIFIC_PARAMS