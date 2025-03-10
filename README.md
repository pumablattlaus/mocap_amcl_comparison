<!-- <p align="center">
  <a href="" rel="noopener">
 <img width=200px height=200px src="https://i.imgur.com/6wj0hh6.jpg" alt="Project logo"></a>
</p> -->

<h3 align="center">AMCL Accuracy</h3>

<div align="center">

[![Status](https://img.shields.io/badge/Status-Inactive-lightgrey.svg)]()
[![GitHub Issues](https://img.shields.io/github/issues/pumablattlaus/mocap_amcl_comparison.svg)](https://github.com/pumablattlaus/mocap_amcl_comparison/issues)
[![GitHub Pull Requests](https://img.shields.io/github/issues-pr/pumablattlaus/mocap_amcl_comparison.svg)](https://github.com/pumablattlaus/mocap_amcl_comparison/pulls)

</div>

---

<p align="center"> Comparing AMCL to Motion Capture in pose accuracy while standing still and in motion (also when map is occluded).
    <br> 
</p>

## 📝 Table of Contents
- [Usage](#usage)
- [LICENSE](#license)
- [TODO](../TODO.md)

## Prerequisites <a name="prerequisites"></a>
- ROS noetic
- [package for lissajous figure](https://github.com/matchRos/diss_Evaluation.git)
- [match_mobile_robotics](https://github.com/match-ROS/match_mobile_robotics/tree/noetic-devel)

## 🎈 Usage <a name="usage"></a>

### Use new map

i.e for mur_a: 	_240116_added_features_
### Launch MuR:

```bash
roslaunch mur_launch_hardware mur620a.launch launch_ur_l:=false launch_ur_r:=false
```

or without external localization?:
```bash
roslaunch mur_launch_hardware mur620a.launch launch_ur_l:=false launch_ur_r:=false external_localization:=false localization_type:=robot_pose
```

### Go to start

```bash
roslaunch mocap_amcl_comparison move_mir_to_start.launch
```

### Launch formation controller
```bash
roslaunch virtual_leader virtual_leader.launch
roslaunch virtual_leader set_leader_pose.launch # set leader pose
roslaunch formation_controller decentralized_leader_follower_control.launch
```

### Lissajous figure
For this this package has to be cloned into ws: https://github.com/matchRos/diss_Evaluation.git
```bash
rosrun dezentralized_controller_tuning lissajous_response_pub.py
```
Alternatively you can publish velocities to */virtual_leader/cmd_vel*.

### All in one launch
```bash
  roslaunch mocap_amcl_comparison formation_control_lissajous.launch
```
### Record data

robot_pose is via bridge, mir_pose_stamped_simple via mocap or map if localization_type:=robot_pose. 
Therefore compare robot_pose to qualisys_map. But robot_pose is without timestamp: change localization type to robot_pose and use qualisys_map and mir_pose_stamped_simple.

```bash
rosbag record /qualisys_map/mur620a/pose /qualisys/mur620a/pose /qualisys/mur620a/velocity /qualisys/mur620a/odom /mur620a/amcl_pose /mur620a/mir_pose_stamped_simple /mur620a/robot_pose /mur620a/cmd_vel
```


## License <a name="license"></a>

See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).
