import pandas as pd
import numpy as np
import math
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List
import json

from find_transformation_mocap import TransformationCalc

from matplotlib import pyplot as plt
# plt.rcParams['text.usetex'] = True
from matplotlib.collections import LineCollection
import bags_paths

def align_data(df1: pd.DataFrame, df2: pd.DataFrame):
    '''Align 2 dataframes based on the timestamps of df1. Uses the mean of df2 values in the range of df1 timestamps.'''
    
    # Initialize a dictionary to hold the mean values
    mean_values = {}

    # Iterate over the df1 index
    for i, t_i in enumerate(df1.index):
        if i > 0:
            # Calculate the lower and upper bounds of the range
            lower_bound = t_i - (t_i - df1.index[i-1]) / 2
            if i < len(df1.index) - 1:
                upper_bound = t_i + (df1.index[i+1] - t_i) / 2
            else:
                upper_bound = t_i + (t_i - df1.index[i-1]) / 2  # Handle the last element edge case
        else:
            # Handle the first element edge case
            lower_bound = 0
            upper_bound = t_i + (df1.index[i+1] - t_i) / 2
            
        # Filter df2 for the current range and calculate the mean
        filtered_df2 = df2[(df2.index > lower_bound) & (df2.index < upper_bound)]
        
        unwrapped_rot = np.unwrap(filtered_df2['rot_z'])
        if any(unwrapped_rot != filtered_df2['rot_z'].values):
            filtered_df2['rot_z'] = unwrapped_rot
            mean_values[t_i] = filtered_df2.mean()
            if abs(mean_values[t_i]['rot_z']) > np.pi:
                mean_values[t_i]['rot_z']-= 2*np.pi*np.sign(mean_values[t_i]['rot_z'])
        else:
           mean_values[t_i] = filtered_df2.mean()
        # remove_outliers() #TODO: for column in filtered_df2.columns
        
        if upper_bound-lower_bound < 0.2*(df1.index[1]-df1.index[0]):
            print(f"Warning: Timestamp {t_i} has a small range of {upper_bound-lower_bound} s")

    # Convert the dictionary to a DataFrame
    df2_aligned = pd.DataFrame.from_dict(mean_values, orient='index')
    df2_aligned.index.name = 'timestamp'
    return df2_aligned

# Function to process PoseStamped or Pose messages
def process_message(msg, data, t):
    try:
        data['timestamp'].append(msg.header.stamp.to_sec())
        pose = msg.pose
    except AttributeError:
        # get time from clock if pose not poseStamped
        data['timestamp'].append(t.to_sec())
        pose=msg
        
    data['x'].append(pose.position.x)
    data['y'].append(pose.position.y)
    data['z'].append(pose.position.z)
    rot_z = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
    data['rot_z'].append(rot_z)
    
def remove_outliers(data: pd.DataFrame, column: str, f: float = 0.99, data_other: List[pd.DataFrame] = None, filter: str = 'iqr'):
    
    if filter == 'quantile':
        #quantile filter: remove the top f% of the data
        q = data[column].quantile(f)
        idx_inliers = data[column] < q
    elif filter == 'iqr':
        # iqr filter: within 2.22 IQR (equiv. to z-score < 3)
        iqr = data.quantile(0.75, numeric_only=False) - data.quantile(0.25, numeric_only=False)
        idx_inliers = np.abs((data - data.median()) / iqr) < 2.22
    else:
        raise ValueError('Unknown filter')

    if data_other is None:
        return data[idx_inliers]
    else:
        return data[idx_inliers], [d[idx_inliers] for d in data_other]
    

def correct_orientation(data: pd.DataFrame):
    # Correct the rotation error to be between -pi and pi
    data['rot_z'][abs(data['rot_z']) > np.pi] -= 2*np.pi*np.sign(data['rot_z'])
    return data
    
def plot_data(data: List[pd.DataFrame], labels: List[str], cm=False):
    fig, axs = plt.subplots(3, 1, sharex=True)
    for i, df in enumerate(data):
        if cm:
            x_data = df['x']*100
            y_data = df['y']*100
        else:
            x_data = df['x']
            y_data = df['y']
        rot_angle = df['rot_z']*180/np.pi
        axs[0].plot(df.index, x_data, label=labels[i])
        axs[1].plot(df.index, y_data, label=labels[i])
        axs[2].plot(df.index, rot_angle, label=labels[i])

    if cm:
        x_label = 'x [cm]'
        y_label = 'y [cm]'
    else:
        x_label = 'x [m]'
        y_label = 'y [m]'
            
    axs[0].set_ylabel(x_label)
    axs[1].set_ylabel(y_label)
    # axs[2].set_ylabel('$\\theta$ [rad]')
    axs[2].set_ylabel('$\\theta$ [°]')
    axs[2].set_xlabel('Time [s]')
    
    if len(labels) > 1:
        axs[0].legend()
        axs[1].legend()
        axs[2].legend()
    return fig

def create_line_collection(df: pd.DataFrame, cmap='viridis'):
    # Create a LineCollection with segments connecting the points
    points = np.column_stack((df['x'], df['y']))
    segments = np.stack((points[:-1], points[1:]), axis=1)
    lc = LineCollection(segments, cmap=cmap, norm=plt.Normalize(df.index.min(), df.index.max()))

    # Set the colormap based on the index values
    lc.set_array(df.index)
    # Set the colormap based on the timestamp values
    # lc.set_array(df.index.to_series().diff().dropna().values)
    return lc  

def calc_vel_acc(df: pd.DataFrame, df_saving: pd.DataFrame = None):
    if df_saving is None:
        df_saving = df
    # velocities in x:
    df["t"] = df.index
    diff_df = df.diff()#.dropna()
    df_saving["v_x"]=diff_df["x"].values/diff_df["t"]   # np.append(diff_df["x"].values/diff_df["t"], np.nan) if dropna is used
    df_saving["v_y"]=diff_df["y"].values/diff_df["t"]
    df_saving["v_rot_z"]=diff_df["rot_z"].values/diff_df["t"]
    df_saving["v"] = np.linalg.norm(diff_df[["x","y"]].values, axis=1) / diff_df["t"]

    # accelerations:
    df_saving['a'] = df['v'].diff()
    df_saving['a_x'] = df['v_x'].diff()
    df_saving['a_y'] = df['v_y'].diff()
    df_saving['a_rot_z'] = df['v_rot_z'].diff()
    
    return df_saving

def extract_timestamps(file_path):
    # Open and read the content of the .txt file
    with open(file_path, 'r') as file:
        content = file.read()
    
    content = content.replace("'", '"') # if single quotes are used in the file
    # Parse the JSON-like structure to a Python dictionary
    data = json.loads(content)

    # Determine which array ('x' or 'rot_z') is longer
    if len(data['x']) >= len(data['rot_z']):
        # Use timestamps from 'x' if 'x' is longer or equal in length to 'rot_z'
        timestamps = data['x']
    else:
        # Use timestamps from 'rot_z' if 'rot_z' is longer
        timestamps = data['rot_z']

    # Extract the first and second timestamps
    t_min = timestamps[0] if len(timestamps) > 0 else 0
    t_max = timestamps[1] if len(timestamps) > 1 else np.inf
    # check if cmd is sent before recording of bag starts not possible without bag time

    return t_min, t_max

def get_distance(df:pd.DataFrame, rotational=False):
    if not rotational:
        path_length = math.sqrt((df[['x', 'y']].iloc[-1] - df[['x', 'y']].iloc[0]).pow(2).sum())
    else:
        unwrapped_rot = np.unwrap(df['rot_z'])
        if any(unwrapped_rot != df['rot_z'].values):
            df['rot_z'] = unwrapped_rot
        path_length = df['rot_z'].iloc[-1] - df['rot_z'].iloc[0]
        
    return path_length

def get_distance_bag(bag_path, t_min, t_max, rotational=False, pose_mocap_topic='/qualisys/mur620b/pose'):
    data = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'rot_z': []}
    with rosbag.Bag(bag_path) as bag:                
        for _, msg, t in bag.read_messages(topics=[pose_mocap_topic]):
            # if isinstance(msg, PoseStamped):
            process_message(msg, data, t)

    # Subtract the first timestamp from all timestamps
    t_min -= data['timestamp'][0]
    t_max -= data['timestamp'][0]
    data['timestamp'] = [t - data['timestamp'][0] for t in data['timestamp']]
    data['timestamp'] = [t - data['timestamp'][0] for t in data['timestamp']]
    
    # check if cmd is sent before recording of bag starts
    if t_min > 3:
        print("cmd_vel is sent before recording of bag starts")
        t_max = t_min
        t_min = 0

    # Convert to DataFrame
    df = pd.DataFrame(data).set_index('timestamp')

    # restrict to t_max
    df = df[t_min < df.index]
    df = df[df.index < t_max]
    
    return get_distance(df, rotational)
    
def get_dataframes(tC, bag_path, pose_amcl_topic='/mur620b/robot_pose', pose_mocap_topic='/qualisys/mur620b/pose'):
    # Initialize data structures
    data1 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'rot_z': []}
    data2 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'rot_z': []}

    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[pose_amcl_topic]):
            # if isinstance(msg, PoseStamped):
            process_message(msg, data1, t)
                
        for _, msg, t in bag.read_messages(topics=[pose_mocap_topic]):
            # if isinstance(msg, PoseStamped):
            process_message(msg, data2, t)

    data1['timestamp'] = [t - data1['timestamp'][0] for t in data1['timestamp']]
    data2['timestamp'] = [t - data2['timestamp'][0] for t in data2['timestamp']]
    
    # Convert to DataFrame
    df1 = pd.DataFrame(data1).set_index('timestamp')
    df2 = pd.DataFrame(data2).set_index('timestamp')
    
    df1 = tC.apply_transformation(df1)
    
    return df1, df2

def get_starting_positions(tC, bag_path, pose_amcl_topic='/mur620b/robot_pose', pose_mocap_topic='/qualisys/mur620b/pose'):
    df1, df2 = get_dataframes(tC, bag_path, pose_amcl_topic, pose_mocap_topic)
    return (df1['x'].mean(), df1['y'].mean(), df1['rot_z'].mean()), (df2['x'].mean(), df2['y'].mean(), df2['rot_z'].mean())
    
def main(tC, bag_path, save_path=None, t_min=0, t_max=np.inf, pose_amcl_topic='/mur620b/robot_pose', pose_mocap_topic='/qualisys/mur620b/pose', max_distance=np.inf, rotational=False, df1_start=None, df2_start=None):

    if save_path is None:
        save_path = bag_path

    # Get the dataframes
    df1, df2 = get_dataframes(tC, bag_path, pose_amcl_topic, pose_mocap_topic)    
    
    # Subtract the first timestamp from all timestamps
    # t_min -= data1['timestamp'][0]
    # t_max -= data1['timestamp'][0]
    t_min -= df1.index[0]
    t_max -= df1.index[0]
    
    # check if cmd is sent before recording of bag starts
    if t_min > 3:
        print("cmd_vel is sent before recording of bag starts")
        t_max = t_min
        t_min = 0


    # restrict to t_max
    df1 = df1[t_min < df1.index]
    df1 = df1[df1.index < t_max]
    df2 = df2[t_min < df2.index]
    df2 = df2[df2.index < t_max]
        
    
    # remove late data (small dt -> large velocity
    try:
        dt_min = (df1.iloc[-1].name - df1.iloc[0].name)/len(df1)*0.5
    except IndexError:
        dt_min = 0
    # df1['dt'] = df1.index.to_series().diff()
    # df1 = df1[df1['dt'] > dt_min]
    # df1.drop(columns=['dt'], inplace=True)
    for i, t_i in enumerate(df1.index):
        if i < len(df1.index) - 1:
            if df1.index[i+1] - t_i < dt_min:
                # remove the data
                df1 = df1.drop(t_i)

    # Resample or align the data
    # Example: Align df2 to df1's timestamps using nearest method
    # df2_aligned = df2.reindex(df1.index, method='nearest')
    df2_aligned = align_data(df1, df2)

    # df1 = tC.apply_transformation(df1)
    # df1=correct_orientation(df1)
    # df2=correct_orientation(df2)

    # rot_z: make continuous without jumps of 2pi between two neighbor values
    df1['rot_z'] = np.unwrap(df1['rot_z'])
    df2_aligned['rot_z'] = np.unwrap(df2_aligned['rot_z'])


    # Subtract starting position
    df1_0 = df1.iloc[0]
    if df1_start is not None:
        df1_0['x'], df1_0['y'], df1_0['rot_z'] = df1_start
    df2_0 = df2_aligned.iloc[0]
    if df2_start is not None:
        df2_0['x'], df2_0['y'], df2_0['rot_z'] = df2_start
    
    df1_wo_start = df1 - df1_0
    df2_aligned_wo_start = df2_aligned - df2_0
    
    # Get legth of the path:
    if not rotational:
        df2_aligned_wo_start['length'] = np.sqrt(df2_aligned_wo_start['x']**2 + df2_aligned_wo_start['y']**2)
    else:
        df2_aligned_wo_start['length'] = df2_aligned_wo_start['rot_z']
    
    # Get all idx < max distance:
    idxs_max_dist = df2_aligned_wo_start['length'] < max_distance
    
    # Subtract df1 from df2
    error = df2_aligned - df1

    df2_corrected = df2_aligned - error.mean()

    df2_aligned=calc_vel_acc(df2_aligned)
    df2_aligned_wo_start=calc_vel_acc(df2_aligned_wo_start)

    # Remove outliers in velocity
    # data_other = [df1, df1_wo_start, df2_corrected]
    # df2_aligned_wo_start, data_other=remove_outliers(df2_aligned_wo_start, 'v', 5, data_other)
    # df1, df1_wo_start, df2_corrected = data_other


    # error_wo_start
    error_wo_start = df2_aligned_wo_start - df1_wo_start

    # disregard ouliers in rot_z:
    error_wo_start['rot_z'][abs(error_wo_start['rot_z']) > np.pi/2] = np.nan
    print(f"{error_wo_start.mean()=}")

    df2_wo_start_corrected = df2_aligned_wo_start - error_wo_start.mean()

    error_corrected = df2_corrected - df1
    error_wo_start_corrected = df2_wo_start_corrected - df1_wo_start
    # disregard ouliers in rot_z:
    error_wo_start_corrected['rot_z'][abs(error_wo_start_corrected['rot_z']) > np.pi/2] = np.nan

    # Plot the result and the original data. One figure for x, one for y, one for rot_z
    # fig=plot_data([df1_wo_start, df2_aligned_wo_start], ['AMCL', 'MoCap'])
    # fig.savefig(save_path+'_xyrot.png')
    
    fig=plot_data([error_wo_start], ['error'])
    fig.savefig(save_path+'_error.svg')
    
    fig=plot_data([error_wo_start], ['error'], cm=True)
    fig.savefig(save_path+'_error_cm.svg')

    # fig=plot_data([error_wo_start_corrected], ['corrected error'])
    # fig.savefig(save_path+'_error_corrected.png')

    # # Plot total error in x,y and error in rot_z
    error_wo_start_corrected['xy'] = np.sqrt(error_wo_start_corrected['x']**2 + error_wo_start_corrected['y']**2)
    # fig, axs = plt.subplots(2, 1, sharex=True)
    # axs[0].plot(error_wo_start_corrected.index, error_wo_start_corrected['xy'], label=None)
    # axs[0].set_ylabel('$e_{trans} [m]$')
    # axs[1].plot(error_wo_start_corrected.index, error_wo_start_corrected['rot_z'], label=None)
    # axs[1].set_ylabel('$e_{rot} [rad]$')
    # fig.savefig(save_path+'_error_corrected_xy.png')
    
    error_wo_start['xy'] = np.sqrt(error_wo_start['x']**2 + error_wo_start['y']**2)
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(df2_aligned_wo_start['length'], error_wo_start['xy'], label=None)
    axs[0].set_ylabel('$e_{trans} [m]$')
    axs[1].plot(df2_aligned_wo_start['length'], error_wo_start['rot_z'], label=None)
    axs[1].set_ylabel('$e_{rot} [rad]$')
    if rotational:
        axs[1].set_xlabel('Path length [rad]')
    else:
        axs[1].set_xlabel('Path length [m]')
    fig.savefig(save_path+'_error_xy_length.svg')
    
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(error_wo_start.index, error_wo_start['xy'], label=None)
    axs[0].set_ylabel('$e_{trans} [m]$')
    axs[1].plot(error_wo_start.index, error_wo_start['rot_z'], label=None)
    axs[1].set_ylabel('$e_{rot} [rad]$')
    axs[1].set_xlabel('Time [s]')
    fig.savefig(save_path+'_error_xy.svg')
    
    error['xy'] = np.sqrt(error['x']**2 + error['y']**2)

    # output the statistics to a file:
    error_corrected_max_d = error_wo_start_corrected[idxs_max_dist]
    error_stats = pd.concat([error_corrected_max_d.mean(), error_corrected_max_d.median(), error_corrected_max_d.std(), error_corrected_max_d.min(), error_corrected_max_d.max(), error_corrected_max_d.pow(2).mean().pow(1/2)], axis=1)
    error_stats.columns = ['Mean', 'Median', 'Standard deviation', 'Min', 'Max', 'RMS']
    error_stats.to_csv(save_path+'_first_interval_error_stats_corrected_max_dist.csv')
    
    error_wo_start_max_d = error_wo_start[idxs_max_dist]
    error_uncorrected_stats = pd.concat([error_wo_start_max_d.mean(), error_wo_start_max_d.median(), error_wo_start_max_d.std(), error_wo_start_max_d.min(), error_wo_start_max_d.max(), error_wo_start_max_d.pow(2).mean().pow(1/2)], axis=1)
    error_uncorrected_stats.columns = ['Mean', 'Median', 'Standard deviation', 'Min', 'Max', 'RMS']
    error_uncorrected_stats.to_csv(save_path+'_first_interval_error_stats_max_dist.csv')

    df2_aligned_wo_start_max_d = df2_aligned_wo_start[idxs_max_dist]
    df2_stats=pd.concat([df2_aligned_wo_start_max_d.mean(), df2_aligned_wo_start_max_d.median(), df2_aligned_wo_start_max_d.std(), df2_aligned_wo_start_max_d.min(), df2_aligned_wo_start_max_d.max(), df2_aligned_wo_start_max_d.pow(2).mean().pow(1/2)], axis=1)
    df2_stats.columns = ['Mean', 'Median', 'Standard deviation', 'Min', 'Max', 'RMS']
    df2_stats.to_csv(save_path+'_first_interval_df2_stats_max_dist.csv')
    print('Done')
    
    return (df1.iloc[0]['x'], df1.iloc[0]['y'], df1.iloc[0]['rot_z']), (df2_aligned.iloc[0]['x'], df2_aligned.iloc[0]['y'], df2_aligned.iloc[0]['rot_z'])


if __name__ == '__main__':
    bag_path_transformation='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
    save_path=None
    t_max=np.inf
    bag_paths=[]
    save_paths=[]

    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
    # t_max=780 # !
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag'
    # t_max=392
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.052_one.bag'
    # t_max=197
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag'
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_two.bag'
    # t_max=223
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.104_one.bag' #!

    # bag_paths, save_paths = bags_paths.get_linear_paths()
    # bag_paths, save_paths = bags_paths.get_mix_paths()
    # bag_paths, save_paths = bags_paths.get_static_linear_paths()
    # start_bag_path = bags_paths.get_start_translation_pose_bag()
    # rotational = False
    bag_paths, save_paths = bags_paths.get_rotation_paths()
    # bag_paths, save_paths = bags_paths.get_static_rotation_paths()
    start_bag_path = bags_paths.get_start_rotation_pose_bag()
    rotational = True

    pose_mocap_topic='/qualisys/mur620b/pose'
    # pose_amcl_topic='/mur620c/mir_pose_stamped_simple'
    pose_amcl_topic='/mur620b/robot_pose'

    # Get fixed transformation from mocap to amcl
    tC = TransformationCalc(bag_path_transformation, '/qualisys/mur620c/pose', '/mur620c/robot_pose')
    tC.calculate_transformation()

    # accel_idxs = slice(0, 7)
    # rotational = False
    accel_idxs = slice(0, None)
    
    t_mins = []
    t_maxs = []
    for bag_path, save_path in zip(bag_paths[accel_idxs], save_paths[accel_idxs]):
        # read changes file for t_min, t_max:
        t_min, t_max = extract_timestamps(save_path+'_changes.txt')
        t_mins.append(t_min)
        t_maxs.append(t_max)

    # Rename save_paths to save_paths_min_dist
    for i, (bag_path, save_path) in enumerate(zip(bag_paths, save_paths)):
        name = save_path.split('/')[-1]
        path=save_path.split(name)[0]+'min_dist_from_start/'
        save_paths[i] = path+name

    min_distance = np.inf
    for i, bag_path in enumerate(bag_paths):
        distance = get_distance_bag(bag_path, 0, np.inf, rotational=rotational)
        print(f"distance {i}: {distance}")
        if distance < min_distance:
            min_distance = distance

    print(f"min_distance: {min_distance}")
    
    t_mins_all = [0 for _ in bag_paths]
    t_maxs_all = [np.inf for _ in bag_paths]

    # df1_0, df2_0 = get_starting_positions(tC, start_bag_path)
    df1_0, df2_0 = None, None
    for bag_path, save_path, t_min, t_max in zip(bag_paths, save_paths, t_mins_all, t_maxs_all):
        # _, _= main(tC, bag_path, save_path, t_min, np.inf, pose_amcl_topic, pose_mocap_topic, min_distance, rotational=rotational, df1_start=df1_0, df2_start=df2_0)
        df1_0, df2_0 = main(tC, bag_path, save_path, t_min, np.inf, pose_amcl_topic, pose_mocap_topic, min_distance, rotational=rotational, df1_start=df1_0, df2_start=df2_0)