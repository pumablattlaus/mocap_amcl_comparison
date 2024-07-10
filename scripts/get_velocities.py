import pandas as pd
import numpy as np
import math
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List
import copy

from find_transformation_mocap import TransformationCalc

from matplotlib import pyplot as plt
# plt.rcParams['text.usetex'] = True
from matplotlib.collections import LineCollection
import bags_paths

# Function to process TwistStamped or Twist messages
def process_message(msg, data, t):
    try:
        data['timestamp'].append(msg.header.stamp.to_sec())
        twist = msg.twist
    except AttributeError:
        # get time from clock if pose not poseStamped
        data['timestamp'].append(t.to_sec())
        twist=msg
        
    data['x'].append(twist.linear.x)
    data['y'].append(twist.linear.y)
    data['z'].append(twist.linear.z)
    data['v'].append(math.sqrt(twist.linear.x**2+twist.linear.y**2+twist.linear.z**2))
    data['rot_z'].append(twist.angular.z)
    
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
    
def plot_data(data: List[pd.DataFrame], labels: List[str]):
    fig, axs = plt.subplots(3, 1, sharex=True)
    for i, df in enumerate(data):
        axs[0].plot(df.index, df['x'], label=labels[i])
        axs[1].plot(df.index, df['y'], label=labels[i])
        axs[2].plot(df.index, df['rot_z'], label=labels[i])

    # axs[0].set_title('x [m]')
    # axs[1].set_title('y [m]')
    # # axs[1].legend()
    # axs[2].set_title('$\\theta$ [rad]')
    # # axs[2].legend()
    axs[0].set_ylabel('x [m]')
    axs[1].set_ylabel('y [m]')
    axs[2].set_ylabel('$\\theta$ [rad]')
    axs[2].set_xlabel('Time [s]')
    
    if len(labels) > 1:
        axs[0].legend()
        axs[1].legend()
        axs[2].legend()
    return fig

def calc_acc(df: pd.DataFrame, df_saving: pd.DataFrame = None):
    if df_saving is None:
        df_saving = df
    # accelerations:
    df["t"] = df.index
    diff_df = df.diff()#.dropna()
    df_saving["a_x"]=diff_df["x"].values/diff_df["t"]   # np.append(diff_df["x"].values/diff_df["t"], np.nan) if dropna is used
    df_saving["a_y"]=diff_df["y"].values/diff_df["t"]
    df_saving["a_rot_z"]=diff_df["rot_z"].values/diff_df["t"]
    df_saving["a"] = diff_df["v"].values/diff_df["t"]
    
    return df_saving

def main(tC, bag_path, save_path=None, t_max=np.inf, velocity_topic='/qualisys/mur620b/velocity'):

    # Initialize data structures
    data1 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'v': [], 'rot_z': []}

    if save_path is None:
        save_path = bag_path

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=[velocity_topic]):
            # if isinstance(msg, PoseStamped):
            process_message(msg, data1, t)
              
    # Subtract the first timestamp from all timestamps
    data1['timestamp'] = [t - data1['timestamp'][0] for t in data1['timestamp']]

    # Convert to DataFrame
    df1 = pd.DataFrame(data1).set_index('timestamp')

    # restrict to t_max
    df1 = df1[df1.index < t_max]
    
    # remove late data (small dt -> large acceleration)
    try:
        dt_min = (df1.iloc[-1].name - df1.iloc[0].name)/len(df1)*0.5
        for i, t_i in enumerate(df1.index):
            if i < len(df1.index) - 1:
                if df1.index[i+1] - t_i < dt_min:
                    # remove the data
                    df1 = df1.drop(t_i)
    except IndexError:
        print(f"Index error for {bag_path}")
        return

    df1 = calc_acc(df1)

    ############### PLOT
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(df1.index, df1['v'])
    axs[1].plot(df1.index, df1['rot_z'])
    axs[0].set_ylabel('v [m/s]')
    axs[1].set_ylabel('$v_\\theta$ [rad/s]')
    axs[1].set_xlabel('Time [s]')
    fig.savefig(save_path+'mocap_vel.png')

    # Plot accelerations
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(df1.index, df1['a'])
    axs[1].plot(df1.index, df1['a_rot_z'])
    axs[0].set_ylabel('a [m/s²]')
    axs[1].set_ylabel('$a_\\theta$ [rad/s²]')
    axs[1].set_xlabel('Time [s]')
    fig.savefig(save_path+'mocap_acel.png')

    ## Rolling window:
    window=15
    df1['v_rolling']=df1['v'].rolling(window).mean()
    df1['rot_rolling']=df1['rot_z'].rolling(window).mean()
    diff_df = df1.diff()#.dropna()
    df1["a_rot_rolling"]=diff_df["rot_rolling"].values/diff_df["t"]
    df1["a_rolling"] = diff_df["v_rolling"].values/diff_df["t"]

    ############### PLOT ROLLING
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(df1.index, df1['v_rolling'])
    axs[1].plot(df1.index, df1['rot_rolling'])
    axs[0].set_ylabel('v [m/s]')
    axs[1].set_ylabel('$v_\\theta$ [rad/s]')
    axs[1].set_xlabel('Time [s]')
    fig.savefig(save_path+'mocap_vel_rolling.png')

    # Plot accelerations
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(df1.index, df1['a_rolling'])
    axs[1].plot(df1.index, df1['a_rot_rolling'])
    axs[0].set_ylabel('a [m/s²]')
    axs[1].set_ylabel('$a_\\theta$ [rad/s²]')
    axs[1].set_xlabel('Time [s]')
    fig.savefig(save_path+'mocap_acel_rolling.png')

    # plt.show()
    plt.close()

    # output the statistics to a file:
    df2_stats=pd.concat([df1.mean(), df1.median(), df1.std(), df1.min(), df1.max(), df1.pow(2).mean().pow(1/2)], axis=1)
    df2_stats.columns = ['Mean', 'Median', 'Standard deviation', 'Min', 'Max', 'RMS']
    df2_stats.to_csv(save_path+'_mocap_vel_stats.csv')
    print('Done')


if __name__ == '__main__':
    bag_path_transformation='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
    save_path=None
    t_max=np.inf
    bag_paths=[]
    save_paths=[]

    bags_lissajous, save_paths_lissajous, t_maxs = bags_paths.get_lissajous_paths()

    ## acceleration experiments: rotation
    # max rot speed set to 0.5:
    bag_paths, save_paths = bags_paths.get_acceleration_paths()    

    pose_mocap_topic='/qualisys/mur620b/pose'
    # pose_amcl_topic='/mur620c/mir_pose_stamped_simple'
    pose_amcl_topic='/mur620b/robot_pose'

    # Get fixed transformation from mocap to amcl
    tC = TransformationCalc(bag_path_transformation, '/qualisys/mur620c/pose', '/mur620c/robot_pose')
    tC.calculate_transformation()

    for bag_path, save_path in zip(bag_paths, save_paths):
        main(tC, bag_path, save_path, t_max)
    
    for bag_path, save_path, t_max in zip(bags_lissajous, save_paths_lissajous, t_maxs):
        main(tC, bag_path, save_path, t_max)
    