import os
import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
from copy import deepcopy
import rosbag
import tf
import csv
from tqdm import tqdm
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

from generator import GenerateRosbag
from transformations import *

class ROSbag_generator():

    def __init__(self, data_path):
        
        self.dataset_path = os.path.abspath(data_path)
    
    def converter(self):

        n_data = len(os.listdir(self.dataset_path + "/images/image/"))

        intrinsics = self.intrinsics_from_ini_file(self.dataset_path + "/intrinsics/asus.ini")

        poses = []

        with open(self.dataset_path + "/trajectory.log", 'r') as file:
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

        pose_1 = []
        pose_2 = []
        pose_3 = []

        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        Rx = rotation_matrix(np.radians(90), xaxis)
        Ry = rotation_matrix(np.radians(90), yaxis)
        Rz = rotation_matrix(np.radians(-90), zaxis)

        rb = GenerateRosbag()
        rosbag_path = os.path.dirname(os.path.abspath(__file__)) + "/../to_ros/ROS1_bags/"
        bag = rosbag.Bag(rosbag_path + self.dataset_path.split("/")[-1] + ".bag", "w")

        for i in tqdm(range(min(n_data, len(poses)))):
            
            header = Header()
            header.seq = i
            header.stamp.secs = i
            header.stamp.nsecs = 0
            header.frame_id = "camera"

            img_rgb = cv2.imread(self.dataset_path + "/images/image/image{:05d}".format(i+1) + ".png", -1)
            img_depth = cv2.imread(self.dataset_path + "/images/depth/depth{:05d}".format(i+1) + ".png", cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH) / 1000.
            
            #pose = convert_4x4_matrix_robotics_to_cv(np.array(matrix_list))
            pose = Rx @ poses[i] @ np.linalg.inv(Ry @ Rz)
            #pose = np.linalg.inv(poses[i])

            pose_quat = quaternion_from_matrix(pose)
            pose_t = pose[:3,3].reshape(-1)
            pose = [*pose_quat] + [*pose_t]

            pose_1.append(pose_t[0])
            pose_2.append(pose_t[1])
            pose_3.append(pose_t[2])

            rgb_msg = rb.create_image_msg(img_rgb, deepcopy(header))
            depth_msg = rb.create_image_msg(img_depth, deepcopy(header))
            cam_info_msg = rb.create_camera_info_msg(img_rgb, intrinsics, deepcopy(header))
            pose_msg = rb.create_pose_msg(pose, deepcopy(header))
            #tf_msg = rb.create_tf_msg(pose, deepcopy(header))


            bag.write("camera/rgb", rgb_msg)
            bag.write("camera/depth", depth_msg)
            bag.write("camera/camera_info", cam_info_msg)
            #bag.write("tf", tf_msg)
            bag.write("amcl_pose", pose_msg)

        bag.close()

        print("ROS1 bag saved in: {}".format(rosbag_path + self.dataset_path.split("/")[-1] + ".bag"))

    
    @staticmethod
    def convert_4x4_matrix_robotics_to_cv(matrix):
        """
        Converts a 4x4 transformation matrix from robotics convention
        (X forward, Y left, Z up) to computer vision convention
        (X right, Y down, Z forward).
        
        Args:
        matrix (np.ndarray): A 4x4 transformation matrix.
        
        Returns:
        np.ndarray: The converted 4x4 transformation matrix.
        """
        # Rotation matrix to convert from robotics to computer vision convention
        R_conversion = np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])

        # Extract the 3x3 rotation part and the 3x1 translation part
        R_robotics = matrix[:3, :3]
        t_robotics = matrix[:3, 3]

        # Convert the rotation matrix
        R_cv = R_conversion @ R_robotics

        # Convert the translation vector
        t_cv = R_conversion @ t_robotics

        # Construct the new 4x4 transformation matrix
        matrix_cv = np.eye(4)
        matrix_cv[:3, :3] = R_cv
        matrix_cv[:3, 3] = t_cv

        return matrix_cv

    @staticmethod
    def convert_4x4_matrix_cv_to_robotics(matrix):
        """
        Converts a 4x4 transformation matrix from computer vision convention
        (X right, Y down, Z forward) to robotics convention
        (X forward, Y left, Z up).
        
        Args:
        matrix (np.ndarray): A 4x4 transformation matrix.
        
        Returns:
        np.ndarray: The converted 4x4 transformation matrix.
        """
        # Inverse rotation matrix to convert from computer vision to robotics convention
        R_conversion = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        # Extract the 3x3 rotation part and the 3x1 translation part
        R_cv = matrix[:3, :3]
        t_cv = matrix[:3, 3]

        # Convert the rotation matrix
        R_robotics = R_conversion @ R_cv @ R_conversion.T

        # Convert the translation vector
        t_robotics = R_conversion @ t_cv

        # Construct the new 4x4 transformation matrix
        matrix_robotics = np.eye(4)
        matrix_robotics[:3, :3] = R_robotics
        matrix_robotics[:3, 3] = t_robotics

        return matrix_robotics

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
    def intrinsics_from_ini_file(filename):
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
    

# params
parser = argparse.ArgumentParser()
parser.add_argument('--datapath', required=True, help='path to the raw data of the scene')

opt = parser.parse_args()

if __name__ == '__main__':

    rb_generator = ROSbag_generator(opt.datapath)
    rb_generator.converter()