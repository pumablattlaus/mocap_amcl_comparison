import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import transforms3d
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List

class TransformationCalc:
    def __init__(self, bag_path, pose_mocap_topic, pose_amcl_topic):
        self.bag_path = bag_path
        self.pose_mocap_topic = pose_mocap_topic
        self.pose_amcl_topic = pose_amcl_topic
        self.data1 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}
        self.data2 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}
        self.rotation_matrix_avg = None
        self.translation_vector = None
        self.rpy = None
# Function to process PoseStamped messages    
    def process_message(self, msg, data, t):
        try:
            data['timestamp'].append(msg.header.stamp.to_sec())
            pose = msg.pose
        except AttributeError:
            data['timestamp'].append(t.to_sec())
            pose = msg
        data['x'].append(pose.position.x)
        data['y'].append(pose.position.y)
        data['z'].append(pose.position.z)
        data['qx'].append(pose.orientation.x)
        data['qy'].append(pose.orientation.y)
        data['qz'].append(pose.orientation.z)
        data['qw'].append(pose.orientation.w)
    
    def quaternion_to_matrix(self, q):
        """Convert a quaternion into a rotation matrix."""
        return R.from_quat(q).as_matrix()
    
    def compute_centroid(self, df):
        """Compute the centroid of points in a DataFrame."""
        return df[['x', 'y', 'z']].mean().values
    
    def align_centroids(self, df_source, df_target):
        """Compute the translation required to align centroids."""
        centroid_source = self.compute_centroid(df_source)
        centroid_target = self.compute_centroid(df_target)
        return centroid_target - centroid_source
    
    def compute_rotation_matrix(self, df_source, df_target):
        """Compute the rotation matrix required to align orientations."""
        rotation_matrices_source = df_source[['qx', 'qy', 'qz', 'qw']].apply(self.quaternion_to_matrix, axis=1)
        rotation_matrices_target = df_target[['qx', 'qy', 'qz', 'qw']].apply(self.quaternion_to_matrix, axis=1)
        target_matrix = rotation_matrices_target.iloc[0]
        source_matrix = rotation_matrices_source.iloc[0]
        rotation_alignment = np.dot(target_matrix, source_matrix.T)
        return rotation_alignment
    
    def compute_average_rotation(self, df_source, df_target):
        """Compute the average rotation matrix required to align orientations."""
        rotation_matrices_source = df_source[['qx', 'qy', 'qz', 'qw']].apply(lambda q: self.quaternion_to_matrix([q['qx'], q['qy'], q['qz'], q['qw']]), axis=1)
        rotation_matrices_target = df_target[['qx', 'qy', 'qz', 'qw']].apply(lambda q: self.quaternion_to_matrix([q['qx'], q['qy'], q['qz'], q['qw']]), axis=1)
        # Compute the average rotation matrix by averaging all relative rotations
        relative_rotations = [np.dot(t, s.T) for s, t in zip(rotation_matrices_source, rotation_matrices_target)]
        average_rotation = np.mean(relative_rotations, axis=0)
        return average_rotation
    
    def apply_transformation(self, df, rotation_matrix=None, translation_vector=None):
        """Apply the rotation and translation to the DataFrame."""
        if rotation_matrix is None:
            rotation_matrix = self.rotation_matrix_avg
        if translation_vector is None:
            translation_vector = self.translation_vector
            
        transformed_poses = []
        for index, row in df.iterrows():
            point = row[['x', 'y', 'z']].values
            # rotated_point = np.dot(rotation_matrix, point)
            # transformed_point = rotated_point + translation_vector
            point = point + translation_vector
            transformed_point = np.dot(rotation_matrix, point)
            
            # rotate the orientation
            try:
                rpy = transforms3d.euler.mat2euler(rotation_matrix)
                theta = row['rot_z'] + rpy[2]
                theta = (theta + np.pi) % (2 * np.pi) - np.pi
            except KeyError:
                theta = 0
            
            transformed_poses.append([transformed_point[0], transformed_point[1], transformed_point[2], theta])
            
            
        # return pd.DataFrame(transformed_poses, columns=['x', 'y', 'z', 'rot_z'])
        # Keep all other columns:
        df[['x', 'y', 'z', 'rot_z']] = np.array(transformed_poses)
        return df
    
    def apply_transformation_to_xytheta(self, df, rotation_matrix=None, translation_vector=None):
        """Apply the rotation and translation to the DataFrame."""
        if rotation_matrix is None:
            rotation_matrix = self.rotation_matrix_avg
        if translation_vector is None:
            translation_vector = self.translation_vector
            
        transformed_poses = []
        for index, row in df.iterrows():
            point = row[['x', 'y']].values
            rotated_point = np.dot(rotation_matrix[:2,:2], point)
            transformed_point = rotated_point + translation_vector[:2]
            
            # rotate the orientation
            rpy = transforms3d.euler.mat2euler(rotation_matrix)
            theta = row['rot_z'] + rpy[2]
            theta = (theta + np.pi) % (2 * np.pi) - np.pi
            
            transformed_poses.append([transformed_point[0], transformed_point[1], theta])
            
        return pd.DataFrame(transformed_poses, columns=['x', 'y', 'rot_z'])
    
    def calculate_transformation(self):
        with rosbag.Bag(self.bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=[self.pose_amcl_topic]):
                self.process_message(msg, self.data1, t)
            for topic, msg, t in bag.read_messages(topics=[self.pose_mocap_topic]):
                self.process_message(msg, self.data2, t)
        
        # Convert to DataFrame
        df1 = pd.DataFrame(self.data1).set_index('timestamp')
        df2 = pd.DataFrame(self.data2).set_index('timestamp')
        
        # Resample or align the data
        # Example: Align df2 to df1's timestamps using nearest method
        df2_aligned = df2.reindex(df1.index, method='nearest')
        
        # Compute the translation vector to align centroids
        self.translation_vector = self.align_centroids(df1, df2_aligned)
        
        # Compute the rotation matrix to align orientations
        # rotation_matrix = self.compute_rotation_matrix(df1, df2_aligned)
        self.rotation_matrix_avg = self.compute_average_rotation(df1, df2_aligned)
        
        # Apply the transformation to df_amcl to align it with df_ref
        df_aligned = self.apply_transformation(df1, self.rotation_matrix_avg, self.translation_vector)
        
        print(self.translation_vector)
        # print(rotation_matrix)
        print(f"Avg Rot: {self.rotation_matrix_avg}")
        
        # rotation mat as rpy
        self.rpy = transforms3d.euler.mat2euler(self.rotation_matrix_avg)
        print(f"{self.rpy=}")
        
        # return self.translation_vector, self.rotation_matrix_avg, self.rpy

if __name__ == "__main__":
    bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
    pose_mocap_topic='/qualisys/mur620c/pose'
    # pose_amcl_topic='/mur620c/mir_pose_stamped_simple'
    pose_amcl_topic='/mur620c/robot_pose'
    tc = TransformationCalc(bag_path, pose_mocap_topic, pose_amcl_topic)
    tc.calculate_transformation()