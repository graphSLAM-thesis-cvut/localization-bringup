# Master's thesis repository on Graph SLAM

For now, the repository consists of two independent parts, which are going to be integrated in the future.
## Graph SLAM using only odometry from realsense
![logo](https://github.com/graphSLAM-thesis-cvut/localization-bringup/blob/main/media/odom_slam_logo.jpg) <br/>
This part is just exparimental, and it creates and optimizes Graph slam, taking a smooth realsense odometry and creating a new fator every 5 seconds. Green is an optimized graph-slam path, red arrorws are realsense odometry.

Download bagfile from [here](http://server.seva-hul.com/CVUT/bags/rs.bag):
```
wget http://server.seva-hul.com/CVUT/bags/rs.bag -O bags/rs.bag
```
Build the package and test the incremental odometry using
```
roslaunch thesis odom_slam.launch
```
## LIO-SAM for CTU HUSKY robot setup (described below)

![logo](https://github.com/graphSLAM-thesis-cvut/localization-bringup/blob/main/media/logo.jpg)
### Hardware:
- Ouster lidar
- [X-sens MTi-30 AHRS](https://cz.mouser.com/datasheet/2/693/mti-series-1358510.pdf) 
### Software
Be sure to use [this](https://github.com/graphSLAM-thesis-cvut/LIO-SAM-CTU) custom forked version of LIO-SAM on **noetic** branch (not master).
### How to run
- Run your environment (real robot/ bagfile/ simulation(if you found a way to simulate 9-axis IMU, tell me please!) ).
- See whivh transforms you already have to run correctluy the following steps. I suggest `rosrun rqt_tf_tree rqt_tf_tree`, but there are other ways to check it. 
- Run lio-sam:
```
roslaunch thesis lio_sam.launch
```
Run one of the two following:
- Get the odom->map transformations ( if ypou have some odom->base_link already running ):
```
roslaunch thesis odom_to_map.launch
```
- Get the baselink->...->map transformations ( if you don't have any odom->base_link already running ):
```
roslaunch thesis odom_to_map.launch
```

- Enjoy! (We both know it never works like that, right?) ~Debug, debug, debug!

## Liofr for ouster lidar using native 6-axis IMU

![logo](https://github.com/graphSLAM-thesis-cvut/localization-bringup/blob/main/media/6axis_logo.jpg)
### Hardware:
- Ouster lidar
### Software
Be sure to use [this](https://github.com/graphSLAM-thesis-cvut/liorf-ctu) custom forked version of liorf on **noetic** branch (not master).
### How to run
- Run your environment (real robot/ bagfile/ simulation).
- See which transforms you already have to run correctluy the following steps. I suggest `rosrun rqt_tf_tree rqt_tf_tree`, but there are other ways to check it. 
- Run liorf:
```
roslaunch thesis liorf.launch
```
Run one of the two following:
- Get the odom->map transformations ( if ypou have some odom->base_link already running ):
```
roslaunch thesis odom_to_map.launch
```
- Get the baselink->...->map transformations ( if you don't have any odom->base_link already running ):
```
roslaunch thesis odom_to_map.launch
```
- Get the os_sensor->...->map transformations:
```
roslaunch thesis sensor_to_map.launch
```

#### Note
Before running a bagfile, set use_sim_time to true:
```
roscore
#open a new window
rosparam set use_sim_time true
```
To run a bUgfile and exclude native odometry, I use this:
```
rosbag play --clock bagfilename.bag /tf:=/tf_dev_null
```
Eventhough odometry may stil be kept there, it's not a problem. Just run the the above-metioned right transformation file. <br/>
I personally do not like leaving the native odometry just to be sure that nice results are from the LIO-SAM for sure, not from native odom :)

Bagfiles you can get from here:
- [bagfile](https://drive.google.com/file/d/1aLDQ2m8X-bRqlnpQXYwgNSpAp5E3woYV/view?usp=share_link) with wheeled odometry (odom->base_link) and lidar localization (map->odom)
- [bagfile](https://drive.google.com/file/d/186ZDQBXg3ULFumABJxUKXwNFR4AFaGDF/view?usp=share_link) with wheeled odometry (odom->base_link) only. <br/>
To get the access, contact `bayerja1@fel.cvut.cz`, or ask `hulchvse@cvut.cz` for the bagfiles 