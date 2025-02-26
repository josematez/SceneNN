import warnings
warnings.filterwarnings("ignore", module="numpy")

import os
import sys
import argparse
import numpy as np
import cv2
from copy import deepcopy
from tqdm import tqdm
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros1_noetic import std_msgs__msg__Header as Header
from rosbags.typesys.stores.empty import builtin_interfaces__msg__Time as Time

from generator import GenerateRosbag
from transformations import *

class ROSbag_generator():

    def __init__(self, data_path):
        self.dataset_path = os.path.abspath(data_path)
    
    def converter(self):
        typestore = get_typestore(Stores.ROS1_NOETIC)
        n_data = len(os.listdir(self.dataset_path + "/images/image/"))

        intrinsics = self.intrinsics_from_ini_file(self.dataset_path + "/intrinsics/asus.ini")
        poses = self.poses_from_trajectory_file(self.dataset_path + "/trajectory.log")

        rb = GenerateRosbag()
        rosbag_path = os.path.dirname(os.path.abspath(__file__)) + "/../to_ros/ROS1_bags/"
        bag_path = rosbag_path + self.dataset_path.split("/")[-1] + ".bag"
        self.remove_existing_bag(bag_path)

        with Writer(bag_path) as writer:
            camera_rgb = writer.add_connection("camera/rgb", "sensor_msgs/msg/Image", typestore=typestore)
            camera_depth= writer.add_connection("camera/depth", "sensor_msgs/msg/Image", typestore=typestore)
            camera_info = writer.add_connection("camera/camera_info", "sensor_msgs/msg/CameraInfo", typestore=typestore)
            amcl_pose = writer.add_connection("amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", typestore=typestore)

            origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
            Rx = rotation_matrix(np.radians(90), xaxis)
            Ry = rotation_matrix(np.radians(90), yaxis)
            Rz = rotation_matrix(np.radians(-90), zaxis)

            for i in tqdm(range(min(n_data, len(poses)))):
                
                header = Header(seq=i, stamp=Time(sec=i,nanosec=0), frame_id="camera")

                img_rgb = cv2.imread(self.dataset_path + "/images/image/image{:05d}".format(i+1) + ".png", -1)
                img_depth = cv2.imread(self.dataset_path + "/images/depth/depth{:05d}".format(i+1) + ".png", cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH) / 1000.
                
                pose = Rx @ poses[i] @ np.linalg.inv(Ry @ Rz)

                pose_quat = quaternion_from_matrix(pose)
                pose_t = pose[:3,3].reshape(-1)
                pose = [*pose_quat] + [*pose_t]

                rgb_msg = rb.create_image_msg(img_rgb, deepcopy(header))
                rgb_msg = typestore.serialize_ros1(rgb_msg,"sensor_msgs/msg/Image")

                depth_msg = rb.create_image_msg(img_depth, deepcopy(header))
                depth_msg = typestore.serialize_ros1(depth_msg,"sensor_msgs/msg/Image")

                cam_info_msg = rb.create_camera_info_msg(img_rgb, intrinsics, deepcopy(header))
                cam_info_msg = typestore.serialize_ros1(cam_info_msg,"sensor_msgs/msg/CameraInfo")

                pose_msg = rb.create_pose_msg(pose, deepcopy(header))
                pose_msg = typestore.serialize_ros1(pose_msg,"geometry_msgs/msg/PoseWithCovarianceStamped")


                timestamp = int(i * 1e9)
                writer.write(camera_rgb, timestamp, data = rgb_msg)
                writer.write(camera_depth, timestamp, data = depth_msg)
                writer.write(camera_info, timestamp, data = cam_info_msg)
                writer.write(amcl_pose, i, data = pose_msg)

        print("ROS1 bag saved in: {}".format(rosbag_path + self.dataset_path.split("/")[-1] + ".bag"))

    @staticmethod
    def rotateZ(matrix, degrees):
        theta = np.radians(degrees)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        rotation_matrix_z = np.array([
            [cos_theta, -sin_theta, 0, 0],
            [sin_theta, cos_theta, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        return np.dot(rotation_matrix_z, matrix)
    
    @staticmethod
    def remove_existing_bag(bag_path):
        if os.path.exists(bag_path):
            print("Bag path already exists")
            answer = input("Do you want to remove it? (y/n): ").lower()
            if answer == 'y':
                os.remove(bag_path)
                print("Bag removed")
            if answer == 'n':
                print("Bag not removed")
                sys.exit()
            else:
                print("Invalid input, exiting")
                sys.exit()
    
    @staticmethod
    def intrinsics_from_ini_file(filename : str):
        parameters = {}
        
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip()
                if line:
                    key, value = line.split()
                    parameters[key] = float(value)
        
        fx = parameters.get('fx')
        fy = parameters.get('fy')
        cx = parameters.get('cx')
        cy = parameters.get('cy')
        
        return [fx, fy, cx, cy]
    
    @staticmethod
    def poses_from_trajectory_file(filename : str):
        poses = []
        with open(filename, 'r') as file:
            lines = file.readlines()
            num_lines = len(lines)
            i = 0
            
            while i < num_lines:
                image_id = lines[i].strip()  # Read image ID from the first line of the block
                
                # Read the next 4 lines as the transformation matrix
                transformation_matrix = []
                for j in range(1, 5):
                    line = lines[i + j].strip()
                    matrix_row = np.array([float(x) for x in line.split()])
                    transformation_matrix.append(matrix_row)
                
                # Convert the list of lists to a 4x4 NumPy array
                transformation_matrix = np.array(transformation_matrix)
                
                # Append the block (image ID and transformation matrix) to the list of blocks
                #poses.append((image_id, transformation_matrix))
                poses.append(transformation_matrix)
                
                # Move to the next block of 5 lines
                i += 5

        return poses
    

# params
parser = argparse.ArgumentParser()
parser.add_argument('--datapath', required=True, help='path to the raw data of the scene')

opt = parser.parse_args()

if __name__ == '__main__':

    rb_generator = ROSbag_generator(opt.datapath)
    rb_generator.converter()
