import pandas as pd
import numpy as np
import math
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List

from find_transformation_mocap import TransformationCalc

from matplotlib import pyplot as plt
# plt.rcParams['text.usetex'] = True
from matplotlib.collections import LineCollection

t_max=np.inf
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
# t_max=780 # !
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag'
# t_max=392
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.052_one.bag'
# t_max=197
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag'
bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_two.bag'
t_max=223
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.104_one.bag' #!
bag_path_transformation='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'

pose_mocap_topic='/qualisys/mur620c/pose'
# pose_amcl_topic='/mur620c/mir_pose_stamped_simple'
pose_amcl_topic='/mur620c/robot_pose'

# Get fixed transformation from mocap to amcl
tC = TransformationCalc(bag_path_transformation, pose_mocap_topic, pose_amcl_topic)
tC.calculate_transformation()

# Initialize data structures
data1 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'rot_z': []}
data2 = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'rot_z': []}

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
        q = df2_aligned_wo_start[column].quantile(f)
        idx_inliers = df2_aligned_wo_start[column] < q
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
    diff_df = df.diff().dropna()
    df_saving["v_x"]=np.append(diff_df["x"].values/diff_df["t"],np.nan)
    df_saving["v_y"]=np.append(diff_df["y"].values/diff_df["t"],np.nan)
    df_saving["v_rot_z"]=np.append(diff_df["rot_z"].values/diff_df["t"],np.nan)
    df_saving["v"] = np.append(np.linalg.norm(diff_df[["x","y"]].values, axis=1) / diff_df["t"], np.nan)

    # accelerations:
    df_saving['a'] = df['v'].diff()
    df_saving['a_x'] = df['v_x'].diff()
    df_saving['a_y'] = df['v_y'].diff()
    df_saving['a_rot_z'] = df['v_rot_z'].diff()
    
    return df
    
with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=[pose_amcl_topic]):
        # if isinstance(msg, PoseStamped):
        process_message(msg, data1, t)
            
    for topic, msg, t in bag.read_messages(topics=[pose_mocap_topic]):
        # if isinstance(msg, PoseStamped):
        process_message(msg, data2, t)

# Subtract the first timestamp from all timestamps
data1['timestamp'] = [t - data1['timestamp'][0] for t in data1['timestamp']]
data2['timestamp'] = [t - data2['timestamp'][0] for t in data2['timestamp']]

# Convert to DataFrame
df1 = pd.DataFrame(data1).set_index('timestamp')
df2 = pd.DataFrame(data2).set_index('timestamp')

# restrict to t_max
df1 = df1[df1.index < t_max]
df2 = df2[df2.index < t_max]

# Resample or align the data
# Example: Align df2 to df1's timestamps using nearest method
df2_aligned = df2.reindex(df1.index, method='nearest')

# Print frequencies:
print(f'Frequency AMCL: {1/df1.index.to_series().diff().mode()[0]}')
print(f'Frequency MoCap: {1/df2.index.to_series().diff().mode()[0]}')
print(f'Frequency aligned: {1/df2_aligned.index.to_series().diff().mode()[0]}')

df1 = tC.apply_transformation(df1)
# df1=correct_orientation(df1)
# df2=correct_orientation(df2)

# rot_z: make continuous without jumps of 2pi between two neighbor values
df1['rot_z'] = np.unwrap(df1['rot_z'])
df2_aligned['rot_z'] = np.unwrap(df2_aligned['rot_z'])

# Subtract df1 from df2
error = df2_aligned - df1

df2_corrected = df2_aligned - error.mean()

# Subtract starting position
df1_wo_start = df1 - df1.iloc[0]
df2_aligned_wo_start = df2_aligned - df2_aligned.iloc[0]

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
# fig=plot_data([df1, df2_aligned], ['AMCL', 'MoCap'])
# fig=plot_data([df1_wo_start, df2_aligned_wo_start], ['AMCL', 'MoCap'])
# fig=plot_data([df1, df2_corrected], ['AMCL', 'MoCap'])
fig=plot_data([df1_wo_start, df2_aligned_wo_start], ['AMCL', 'MoCap'])
fig.savefig(bag_path+'_xyrot.png')

# fig=plot_data([error_wo_start], ['error'])
# fig.savefig(bag_path+'_error.png')

fig=plot_data([error_wo_start_corrected], ['corrected error'])
fig.savefig(bag_path+'_error_corrected.png')

# Plot total error in x,y and error in rot_z
error_wo_start_corrected['xy'] = np.sqrt(error_wo_start_corrected['x']**2 + error_wo_start_corrected['y']**2)
fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(error_wo_start_corrected.index, error_wo_start_corrected['xy'], label=None)
axs[0].set_ylabel('$e_{trans} [m]$')
axs[1].plot(error_wo_start_corrected.index, error_wo_start_corrected['rot_z'], label=None)
axs[1].set_ylabel('$e_{rot} [rad]$')
fig.savefig(bag_path+'_error_corrected_xy.png')

# Use lines for the xy plot with color based on time
lc_df1 = create_line_collection(df1, 'winter') #Blues
lc_df2 = create_line_collection(df2_corrected, cmap='copper') #Reds

# Create the scatter plot with a line
fig, ax = plt.subplots()
ax.add_collection(lc_df1)
ax.add_collection(lc_df2)
ax.autoscale()
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.colorbar(lc_df1, label='AMCL: Duration [s]')
plt.colorbar(lc_df2, label='MoCap: Duration [s]')
# plt.show()

fig.savefig(bag_path+'.png')

# Plot velocities total
fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(df2_aligned_wo_start.index, df2_aligned_wo_start['v'])
axs[1].plot(df2_aligned_wo_start.index, df2_aligned_wo_start['v_rot_z'])
axs[0].set_ylabel('v [m/s]')
axs[1].set_ylabel('$v_\\theta$ [rad/s]')
axs[1].set_xlabel('Time [s]')
fig.savefig(bag_path+'_vel.png')

# Plot accelerations
fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(df2_aligned_wo_start.index, df2_aligned_wo_start['a'])
axs[1].plot(df2_aligned_wo_start.index, df2_aligned_wo_start['a_rot_z'])
axs[0].set_ylabel('a [m/s²]')
axs[1].set_ylabel('$a_\\theta$ [rad/s²]')
axs[1].set_xlabel('Time [s]')
fig.savefig(bag_path+'_acel.png')


# plt.show()

# Get all meaningful statistics from the error:
print('Mean')
print(error.mean())
print('Median')
print(error.median())
print('Standard deviation')
print(error.std())
print('Min')
print(error.min())
print('Max')
print(error.max())
print('RMS')
print(error.pow(2).mean().pow(1/2))

print('Corrected error')
print('Mean')
print(error_wo_start_corrected.mean())
print('Median')
print(error_wo_start_corrected.median())
print('Standard deviation')
print(error_wo_start_corrected.std())
print('Min')
print(error_wo_start_corrected.min())
print('Max')
print(error_wo_start_corrected.max())
print('RMS')
print(error_wo_start_corrected.pow(2).mean().pow(1/2))

# output the statistics to a file:
error_stats = pd.concat([error_wo_start_corrected.mean(), error_wo_start_corrected.median(), error_wo_start_corrected.std(), error_wo_start_corrected.min(), error_wo_start_corrected.max(), error_wo_start_corrected.pow(2).mean().pow(1/2)], axis=1)
error_stats.columns = ['Mean', 'Median', 'Standard deviation', 'Min', 'Max', 'RMS']
error_stats.to_csv(bag_path+'_error_stats.csv')

df2_stats=pd.concat([df2_aligned_wo_start.mean(), df2_aligned_wo_start.median(), df2_aligned_wo_start.std(), df2_aligned_wo_start.min(), df2_aligned_wo_start.max(), df2_aligned_wo_start.pow(2).mean().pow(1/2)], axis=1)
df2_stats.columns = ['Mean', 'Median', 'Standard deviation', 'Min', 'Max', 'RMS']
df2_stats.to_csv(bag_path+'_df2_stats.csv')
print('Done')
