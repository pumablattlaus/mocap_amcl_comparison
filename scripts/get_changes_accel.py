import pandas as pd
import numpy as np
import math
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List, Dict


# from .bags_paths import get_bag_paths
import bags_paths

def get_changes(data: Dict, changes: Dict, column_name: str = 'x'):

    t = data['timestamp'][-1]
    data_col = data[column_name]
    try:
        direction=data_col[-2]-data_col[-3]
    except IndexError:
        if len(data_col) == 1:
            # direction=np.sign(data_col[0]) # velocity was 0 at first
            changes[column_name].append(t) # first command time
            return changes# first data
        elif len(data_col) == 2:
             direction=np.sign(data_col[0]) # velocity was 0 at first
        else:
            raise IndexError

    if data_col[-1]>data_col[-2] and direction<0:
        changes[column_name].append(t)
    elif data_col[-1]<data_col[-2] and direction>0:
        changes[column_name].append(t)

    return changes

# Function to process cmd_vel
def process_message(msg, data: Dict, t, changes: Dict):

    # get time from clock if pose not poseStamped
    data['timestamp'].append(t.to_sec())
    cmd_vel=msg
        
    data['x'].append(cmd_vel.linear.x)
    data['y'].append(cmd_vel.linear.y)
    data['z'].append(cmd_vel.linear.z)
    data['lin_ges'].append(math.sqrt(cmd_vel.linear.x**2+cmd_vel.linear.y**2+cmd_vel.linear.z**2))
    data['rot_z'].append(cmd_vel.angular.z)

    changes=get_changes(data, changes, 'x')
    changes=get_changes(data, changes, 'rot_z')

def main(bag_path, save_path, cmd_vel_topic='/mur620b/cmd_vel'):
    if save_path is None:
        save_path = bag_path

    # Initialize data structures
    data = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'lin_ges': [], 'rot_z': []}
    changes = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'lin_ges': [], 'rot_z': []}

    # Output data and changes to a csv:
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=[cmd_vel_topic]):
        process_message(msg, data, t, changes)

    # Save the data to a csv file
    df = pd.DataFrame(data)
    df.to_csv(save_path+'_cmd_vel.csv', index=False)

    # Save the changes to a csv file. Columns dont have same length, so save the dict directly
    # np.save(save_path+'_changes.npy', changes)
    # (save_path+'_changes.csv', index=False)
    with open(save_path+'_changes.txt', 'w') as f:
        f.write(str(changes))

if __name__ == "__main__":
    bags_lissajous=[]
    save_paths_lissajous = []
    t_maxs = []
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.013_one')
    t_maxs.append(np.inf)
    # t_max=780 # !
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.026_one')
    t_maxs.append(780)
    # t_max=392
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.052_one.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.052_one')
    t_maxs.append(392)
    # t_max=197
    bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag'
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.013_two')
    t_maxs.append(197)
    bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_two.bag'
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.026_two')
    t_maxs.append(np.inf)
    # t_max=223
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.104_one.bag' #!
    
    t_max=np.inf
    bag_paths, save_paths = bags_paths.get_acceleration_paths()

    for bag_path, save_path in zip(bag_paths, save_paths):
        main(bag_path, save_path)
    
    for bag_path, save_path, t_max in zip(bags_lissajous, save_paths_lissajous, t_maxs):
        main(bag_path, save_path)
    
