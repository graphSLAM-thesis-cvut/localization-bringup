# LIO-SAM setup for a HUSKY robot with 9-axis IMU and LIDAR
## Hardware:
- Ouster lidar (had to change some LIO-SAM code as suggested in [here](https://github.com/TixiaoShan/LIO-SAM/issues/94), as the drivers may be old)
- [X-sens MTi-30 AHRS](https://cz.mouser.com/datasheet/2/693/mti-series-1358510.pdf) 
## Software
Be sure to use [this](https://github.com/graphSLAM-thesis-cvut/LIO-SAM-CTU) custom forked version of LIO-SAM on **noetic** branch (not master).
## How to run
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
### Note
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