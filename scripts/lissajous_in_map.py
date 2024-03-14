import pandas as pd
import numpy as np
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import tf
from typing import List

from find_transformation_mocap import TransformationCalc

from matplotlib import pyplot as plt
# plt.rcParams['text.usetex'] = True
from matplotlib.collections import LineCollection

# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag'
# bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.052_one.bag'
bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag'
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

def correct_orientation(data: pd.DataFrame):
    # Correct the rotation error to be between -pi and pi
    data['rot_z'][abs(data['rot_z']) > np.pi] -= 2*np.pi*np.sign(data['rot_z'])
    return data
    

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


# Use lines for the xy plot with color based on time
lc_df1 = create_line_collection(df1, 'winter') #Blues
lc_df2 = create_line_collection(df2, cmap='copper') #Reds

# Map image
map_image = plt.imread('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/Scale_240208.png')
image_height, image_width, _ = map_image.shape
scale = 0.05  # 1 pixel = 0.1 m
x_offset = 0.0  # Offset in x 
y_offset = 0.0  # Offset in y

# Extent: left, right, bottom, top
extent = [
    0 - x_offset,  # Left
    image_width * scale - x_offset,  # Right
    0 - y_offset,  # Bottom
    image_height * scale - y_offset  # Top
]

fig, ax = plt.figure(), plt.gca()

# Add the map image
ax.imshow(map_image, extent=extent, zorder=0) # Ensure the image is behind the plots by setting zorder=0



# Create the scatter plot with a line
ax.add_collection(lc_df1)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
plt.colorbar(lc_df1, label='Duration [s]')
ax.autoscale()

plt.show()

fig.savefig(bag_path+'_map.png', bbox_inches='tight')

print('Done')
