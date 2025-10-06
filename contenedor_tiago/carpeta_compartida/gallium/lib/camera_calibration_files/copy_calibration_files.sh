#!/bin/bash

SOURCE_DIR=`rospack find camera_calibration_files`/config
if [ ! -z $pal_camera_calibration_intrinsics_calibration_path ]; then
    SOURCE_DIR="$pal_camera_calibration_intrinsics_calibration_path"
fi
TARGET_DIR="$HOME/.ros/camera_info/"
mkdir -p $TARGET_DIR
cd $SOURCE_DIR
for f in `ls *.yaml`; do
    if [ ! -f $TARGET_DIR/$f ]; then
        echo "Copying configuration from $SOURCE_DIR/$f to $TARGET_DIR/$f"
        cp $SOURCE_DIR/$f $TARGET_DIR/$f
    else
        echo "Not copying configuration from $SOURCE_DIR/$f to $TARGET_DIR/$f because it already exists"
    fi
done
