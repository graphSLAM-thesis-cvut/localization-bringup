#!/usr/bin/env bash

RosBags="lio_sam_shifted.bag liorf_externalOdom1.bag liorf_orig.bag lvi_sam1.bag lvi_t2651.bag t265.bag"
#RosBags="lio_garden.bag lvi_garden.bag"
roscore &
rosparam set use_sim_time true &
rosbag record -a -O merged.bag &

i=0
for value in $RosBags
do
   if [ "$i" -eq "0" ]
   then
       rosbag play --clock -r 3 $value & 
   else
       rosbag play -r 3 $value & 	
   fi
   i=$((i+1))
done
