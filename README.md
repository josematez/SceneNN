This repository aims to facilitate the automatic creation of bags for both, ROS1 and ROS2, of sequences of the [SceneNN](https://hkust-vgd.github.io/scenenn/) dataset. Preview of the available sequences [here](https://hkust-vgd.ust.hk/scenenn/home/webgl/).

# Requeriments

To setup your environment, simply run:

```
pip install -r requirements.txt
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
