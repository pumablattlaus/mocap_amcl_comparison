import pandas as pd
import numpy as np
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List

from find_transformation_mocap import TransformationCalc

from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection

# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag'
bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.052_one.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_two.bag'
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
    
def plot_data(data: List[pd.DataFrame], labels: List[str]):
    fig, axs = plt.subplots(3, 1, sharex=True)
    for i, df in enumerate(data):
        axs[0].plot(df.index, df['x'], label=labels[i])
        axs[1].plot(df.index, df['y'], label=labels[i])
        axs[2].plot(df.index, df['rot_z'], label=labels[i])

    axs[0].set_title('x')
    axs[0].legend()
    axs[1].set_title('y')
    axs[1].legend()
    axs[2].set_title('rot_z')
    axs[2].legend()

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

# Resample or align the data
# Example: Align df2 to df1's timestamps using nearest method
df2_aligned = df2.reindex(df1.index, method='nearest')

# Print frequencies:
print(f'Frequency AMCL: {1/df1.index.to_series().diff().mode()[0]}')
print(f'Frequency MoCap: {1/df2.index.to_series().diff().mode()[0]}')
print(f'Frequency aligned: {1/df2_aligned.index.to_series().diff().mode()[0]}')

df1 = tC.apply_transformation(df1)

# Subtract df1 from df2
error = df2_aligned - df1

df2_corrected = df2_aligned - error.mean()

# Subtract starting position
df1_wo_start = df1 - df1.iloc[0]
df2_aligned_wo_start = df2_aligned - df2_aligned.iloc[0]

# error_wo_start
error_wo_start = df2_aligned - df1
error_corrected = df2_corrected - df1

# Plot the result and the original data. One figure for x, one for y, one for rot_z
plot_data([df1_wo_start, df2_aligned_wo_start], ['AMCL', 'MoCap'])
plot_data([error_wo_start], ['error_wo_start'])
plot_data([error_corrected], ['error_corrected'])

# Also plot x and y in the same figure, with the time as color
plt.figure()
plt.scatter(df1['x'], df1['y'], c=df1.index)
plt.scatter(df2_corrected['x'], df2_corrected['y'], c=df2_corrected.index)
plt.colorbar()
plt.title('x-y plot')
plt.xlabel('x')
plt.ylabel('y')
# plt.show()


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
plt.colorbar(lc_df1, label='Time AMCL')
plt.colorbar(lc_df2, label='Time MoCap')
plt.show()

fig.savefig(bag_path+'.png')

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

print('Done')
