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

from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection

# bag_path='bags/2024-01-22-15-59-08_compare_test_no_movement.bag'
# bag_path='bags/2024-01-22-16-01-55_compare_test_movement.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_two.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag'
bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag'

# pose_mocap_topic='/qualisys_map/mur620c/pose'
# pose_amcl_topic='/mur620c/mir_pose_stamped_simple'
pose_mocap_topic='/qualisys/mur620c/pose'
pose_amcl_topic='/mur620c/robot_pose'

# Initialize data structures
data1 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}
data2 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}

# Function to process PoseStamped messages
def process_message(msg, data, t):
    try:
        data['timestamp'].append(msg.header.stamp.to_sec())
        pose = msg.pose
    except AttributeError:
        # get time from clock
        data['timestamp'].append(t.to_sec())
        pose=msg
    data['x'].append(pose.position.x)
    data['y'].append(pose.position.y)
    data['z'].append(pose.position.z)
    data['qx'].append(pose.orientation.x)
    data['qy'].append(pose.orientation.y)
    data['qz'].append(pose.orientation.z)
    data['qw'].append(pose.orientation.w)
    # rot_z = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
    
def quaternion_to_matrix(q):
    """Convert a quaternion into a rotation matrix."""
    return R.from_quat(q).as_matrix()

def compute_centroid(df):
    """Compute the centroid of points in a DataFrame."""
    return df[['x', 'y', 'z']].mean().values

def align_centroids(df_source, df_target):
    """Compute the translation required to align centroids."""
    centroid_source = compute_centroid(df_source)
    centroid_target = compute_centroid(df_target)
    return centroid_target - centroid_source

def compute_rotation_matrix(df_source, df_target):
    """Compute the rotation matrix required to align orientations."""
    rotation_matrices_source = df_source[['qx', 'qy', 'qz', 'qw']].apply(quaternion_to_matrix, axis=1)
    rotation_matrices_target = df_target[['qx', 'qy', 'qz', 'qw']].apply(quaternion_to_matrix, axis=1)
    
    # Assuming all rotations in df_source should be aligned with the first rotation in df_target for simplicity
    # For a more accurate alignment, consider averaging the rotations or using a more sophisticated method
    target_matrix = rotation_matrices_target.iloc[0]
    
    # Calculate the rotation matrix to align the first source rotation with the target rotation
    source_matrix = rotation_matrices_source.iloc[0]
    rotation_alignment = np.dot(target_matrix, source_matrix.T)
    
    return rotation_alignment

def compute_average_rotation(df_source, df_target):
    """Compute the average rotation matrix required to align orientations."""
    rotation_matrices_source = df_source[['qx', 'qy', 'qz', 'qw']].apply(lambda q: quaternion_to_matrix([q['qx'], q['qy'], q['qz'], q['qw']]), axis=1)
    rotation_matrices_target = df_target[['qx', 'qy', 'qz', 'qw']].apply(lambda q: quaternion_to_matrix([q['qx'], q['qy'], q['qz'], q['qw']]), axis=1)
    
    # Compute the average rotation matrix by averaging all relative rotations
    relative_rotations = [np.dot(t, s.T) for s, t in zip(rotation_matrices_source, rotation_matrices_target)]
    average_rotation = np.mean(relative_rotations, axis=0)
    
    return average_rotation

def apply_transformation(df, rotation_matrix, translation_vector):
    """Apply the rotation and translation to the DataFrame."""
    transformed_points = []
    for index, row in df.iterrows():
        point = row[['x', 'y', 'z']].values
        rotated_point = np.dot(rotation_matrix, point)
        transformed_point = rotated_point + translation_vector
        transformed_points.append(transformed_point)
    
    return pd.DataFrame(transformed_points, columns=['x', 'y', 'z'])

# MAIN: 
with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=[pose_amcl_topic]):
        # if isinstance(msg, PoseStamped):
        process_message(msg, data1, t)
            
    for topic, msg, t in bag.read_messages(topics=[pose_mocap_topic]):
        # if isinstance(msg, PoseStamped):
        process_message(msg, data2, t)
        
# Convert to DataFrame
df1 = pd.DataFrame(data1).set_index('timestamp')
df2 = pd.DataFrame(data2).set_index('timestamp')

# Resample or align the data
# Example: Align df2 to df1's timestamps using nearest method
df2_aligned = df2.reindex(df1.index, method='nearest')

# Compute the translation vector to align centroids
translation_vector = align_centroids(df1, df2_aligned)

# Compute the rotation matrix to align orientations
rotation_matrix = compute_rotation_matrix(df1, df2_aligned)

rotation_matrix_avg = compute_average_rotation(df1, df2_aligned)

# Apply the transformation to df_amcl to align it with df_ref
df_aligned = apply_transformation(df1, rotation_matrix, translation_vector)

print(translation_vector)
print(rotation_matrix)
print("Avg Rot *1000: " + rotation_matrix_avg*1e3)
# rotation mat as rpy
rpy = transforms3d.euler.mat2euler(rotation_matrix)
print(rpy)