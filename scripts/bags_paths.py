def get_acceleration_paths():
    rotation_paths = get_rotation_paths()
    linear_paths = get_linear_paths()
    mix_paths = get_mix_paths()

    bag_paths = rotation_paths[0] + linear_paths[0] + mix_paths[0]
    save_paths = rotation_paths[1] + linear_paths[1] + mix_paths[1]
    return bag_paths, save_paths

def get_rotation_paths():
    bag_paths=[]
    save_paths=[]
    ## acceleration experiments: rotation
    # max rot speed set to 0.5:
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-05-29.bag') # 0.01 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.01_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-07-38.bag') # 0.025 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.025_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-08-37.bag') # 0.05 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.05_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-09-19.bag') # 0.075 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.075_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-10-12.bag') # 0.1 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.1_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-11-05.bag') # 0.125 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.125_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-13-39.bag') # 0.15 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.15_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-14-38.bag') # 0.175 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.175_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-15-54.bag') # 0.2 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.2_max_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-17-14.bag') # 0.25 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.25_max_0.5')

    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-12-00.bag') # 0.5 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_0.5_max_0.5')
    
    bags_static, save_static = get_static_rotation_paths()
    bag_paths += bags_static
    save_paths += save_static
    
    ## acceleration experiments: rotation, jerk
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-28-11.bag') # 0.25 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_jerk_0.25')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-29-40.bag') # 0.1 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_jerk_0.1')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-30-41.bag') # 0.05 rad/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_jerk_0.05')

    return bag_paths, save_paths

def get_static_rotation_paths():
    bag_paths=[]
    save_paths=[]
     ## acceleration experiments: rotation, static velocity
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-13-55-37.bag') # 0.05 rad/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_static_0.05')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-13-53-42.bag') # 0.1 rad/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_static_0.1')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-13-58-03.bag') # 0.25 rad/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_static_0.25')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-13-58-59.bag') # 0.5 rad/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_static_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-13-59-42.bag') # 0.75 rad/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/rotation/rot_static_0.75')
    return bag_paths, save_paths

def get_start_rotation_pose_bag():
    return '/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-27-09.bag'
    
def get_linear_paths():
    bag_paths=[]
    save_paths=[]
    ## acceleration experiments: translation
    # max speed set to 1.0:
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-11-46-04.bag') # 0.01 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.01_max_1.0')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-11-52-16.bag') # 0.1 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.1_max_1.0')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-11-55-06.bag') # 0.15 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.15_max_1.0')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-11-56-36.bag') # 0.2 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.2_max_1.0')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-11-59-26.bag') # 0.25 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.25_max_1.0')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-12-00-43.bag') # 0.25 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.3_max_1.0')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-12-07-05.bag') # 0.25 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_0.3_max_1.0_50Hz')

    ## acceleration experiments: translation, static velocity
    bags_static, save_static = get_static_linear_paths()
    bag_paths += bags_static
    save_paths += save_static

    ## acceleration experiments: translation, jerk
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-32-58.bag') # 0.3 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_jerk_0.3')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-34-38.bag') # 0.2 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_jerk_0.2')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-35-55.bag') # 0.1 m/s²
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_jerk_0.1')
    return bag_paths, save_paths

def get_static_linear_paths():
    bag_paths=[]
    save_paths=[]
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-12-20-03.bag') # 0.25 m/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_static_0.25')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-12-18-53.bag') # 0.5 m/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_static_0.5')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-12-21-29.bag') # 0.75 m/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_static_0.75')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-12-09-33.bag') # 1.0 m/s
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/translation/transl_static_1.0')
    return bag_paths, save_paths

def get_start_translation_pose_bag():
    return '/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-32-19.bag'

def get_mix_paths():
    bag_paths=[]
    save_paths=[]

    ## acceleration experiments: MIX
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-45-30.bag') # 0.05m/s 0.075rad/s² max_lin=1.0 max_ang=0.5
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/mix/mix_0.075rad2_const_lin_0.05')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-47-40.bag') # 0.25m/s 0.075rad/s² max_lin=1.0 max_ang=0.5
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/mix/mix_0.075rad2_const_lin_0.25')
    bag_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/acceleration_experiments_240321/2024-03-21-14-49-54.bag') # 0.5m/s 0.075rad/s² max_lin=1.0 max_ang=0.5
    save_paths.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/acceleration_experiments_240321/mix/mix_0.075rad2_const_lin_0.5')

    return bag_paths, save_paths

def get_lissajous_paths():
    import numpy as np
    
    bags_lissajous=[]
    save_paths_lissajous = []
    t_maxs = []
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_one.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.013_one')
    t_maxs.append(780)
    
    # t_max=780 # !
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_one.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.026_one')
    t_maxs.append(392)
    # t_max=392
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.052_one.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.052_one')
    t_maxs.append(197)
    # t_max=197
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.013_two.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.013_two')
    t_maxs.append(700)
    bags_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.026_two.bag')
    save_paths_lissajous.append('/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/evaluation/mocap_240208_lissajous/lissajous_0.026_two')
    t_maxs.append(220)
    # t_max=223
    # bag_path='/home/rosmatch/hee/amcl_comparison_ws/src/mocap_amcl_comparison/bags/mocap_240208/lissajous_0.104_one.bag' #!
    return bags_lissajous, save_paths_lissajous, t_maxs
