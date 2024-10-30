##!/bin/sh

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <scene_id>"
    exit 1
fi

scene_id=$1;
scene_id=$(printf "%03d" "$scene_id")

THIS_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the scene directory exists
if [ -d "$THIS_FILE_DIR/raw_data/$scene_id" ]; then
    echo "Directory $scene_dir exists. Proceeding with the ROSbag generation..."
   
else
    echo "The raw data of $scene does not exist. Downloading it..."
    sh download.sh $1
fi

python3 $THIS_FILE_DIR/utils/raw_to_rosbag.py --datapath "$THIS_FILE_DIR/raw_data/$scene_id"
rosbags-convert "$THIS_FILE_DIR/to_ros/ROS1_bags/$scene_id.bag"
mv "$THIS_FILE_DIR/to_ros/ROS1_bags/$scene_id" "$THIS_FILE_DIR/to_ros/ROS2_bags/$scene_id"
