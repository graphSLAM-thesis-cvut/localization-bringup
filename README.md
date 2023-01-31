# LIO-SAM setup for a HUSKY robot with 9-axis IMU and LIDAR
## Hardware:
- Ouster lidar (had to change some LIO-SAM code as suggested in [here](https://github.com/TixiaoShan/LIO-SAM/issues/94), as the drivers may be old)
- [X-sens MTi-30 AHRS](https://cz.mouser.com/datasheet/2/693/mti-series-1358510.pdf) 
## Software
Be sure to use [this](https://github.com/graphSLAM-thesis-cvut/LIO-SAM-CTU) custom forked version of LIO-SAM on **noetic** branch (not master).
## How to run
- For now, be sure to turn off anything that publishes tf from odom -> base_link. it's easy to fix, but I commit this version as it works
- Run lio-sam:
```
roslaunch thesis lio_sam.launch
```
- Get the odom/map transformations:
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
Bagfiles you can get from here:
- [bagfile](https://drive.google.com/file/d/1aLDQ2m8X-bRqlnpQXYwgNSpAp5E3woYV/view?usp=share_link) with wheeled odometry (odom->base_link) and lidar localization (map->odom)
- [bagfile](https://drive.google.com/file/d/186ZDQBXg3ULFumABJxUKXwNFR4AFaGDF/view?usp=share_link) with wheeled odometry (odom->base_link) only. <br/>
To get the access, contact `bayerja1@fel.cvut.cz`, or ask `hulchvse@cvut.cz` for the bagfiles 