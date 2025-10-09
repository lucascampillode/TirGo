#!/bin/sh

: ${OROCOS_TARGET:=gnulinux}

for file_path in `echo $CMAKE_PREFIX_PATH | tr : ' '` "/opt/pal/gallium"; do
  if [ ! -d "$file_path/lib/orocos" ]; then
    continue
  elif [ -z "$RTT_COMPONENT_PATH" ]; then
    RTT_COMPONENT_PATH="$file_path/lib/orocos"
  elif ! echo "$RTT_COMPONENT_PATH" | grep -q "$file_path/lib/orocos"; then
    RTT_COMPONENT_PATH="$RTT_COMPONENT_PATH:$file_path/lib/orocos"
  fi
done

export OROCOS_TARGET
export RTT_COMPONENT_PATH
