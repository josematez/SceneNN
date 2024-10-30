This repository aims to facilitate the automatic creation of bags for both, ROS1 and ROS2, of sequences of the SceneNN dataset.

# Requeriments

It requires the rosbags-convert[https://pypi.org/project/rosbags/] package in order to convert from ROS1 to ROS2 bags. To install it:

```
pip install rosbags
```

# Installation

Simply, clone the repository:

```
git clone git@github.com:josematez/SceneNN.git
```

# Usage

To only download the raw data of a given sequence:

```
sh download.sh <sequence_id>
```
Note that, <sequence_id> needs to be substituted by the ID of the sequence in the dataset, for example, for the sequence 011, it is enough just to do: ```sh download.sh 11```

To automatically download (if not already downloaded) and create both ROS1 and ROS2 bags:

```
sh create_rosbag.sh <sequence_id>
```

It will create in the directories to_ros/ROS1_bags and to_ros/ROS2_bags the respective bags files.
