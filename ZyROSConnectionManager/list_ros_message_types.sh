#!/bin/bash

ROS_INSTALL_PREFIX=""
ROS_DISTRO=""

echo "Argument count: $#"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <output_directory> (<ROS_INSTALL_PREFIX> <ROS_DISTRIBUTION>)"
    exit 1
fi

OUTPUT_DIRECTORY=$1

if [ $# == 2 ]; then
     ROS_INSTALL_PREFIX="/opt/ros"
     ROS_DISTRO="kinetic"
else
     if [ $# == 4 ]; then
         ROS_INSTALL_PREFIX=$2
         ROS_DISTRO=$3
     else
         ROS_INSTALL_PREFIX="/opt/ros"
         ROS_DISTRO="kinetic"
     fi
fi

echo "Output directory for generated header files: $OUTPUT_DIRECTORY"
echo "Using ROS install prefix                   : $ROS_INSTALL_PREFIX"
echo "Using ROS version                          : $ROS_DISTRO"

if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "Did not find ROS' setup.bash file under $ROS_INSTALL_PREFIX/$ROS_DISTRO. Please check your ROS installation, or provide the correct install prefix."
    exit 1
fi

if [ -z "$ROS_ROOT" ]; then
    echo "The ROS_ROOT environment variable is not set. Please check your ROS installation!"
    exit 1
fi

if [ ! -f $ROS_ROOT/../../bin/rosmsg ]; then
    echo "rosmsg executable not found."
    exit 1
fi

if [ ! -f $ROS_ROOT/../../bin/rossrv ]; then
    echo "rossrv executable not found."
    exit 1
fi

SCRIPT_BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ ! -f $SCRIPT_BASE_DIR/generate_message_and_service_files.py ]; then
    echo "Python helper script for source file generation not found!"
    exit 1
fi

message_types=$(rosmsg list)
echo "=== Known ROS message types ==="
echo $message_types

service_types=$(rossrv list)
echo "=== Known ROS service types ==="
echo $service_types

echo "Calling Python script for C++ source file generation."

python $SCRIPT_BASE_DIR/generate_message_and_service_files.py "$OUTPUT_DIRECTORY" "$message_types" "$service_types"
