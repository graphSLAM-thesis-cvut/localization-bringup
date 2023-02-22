#!/usr/bin/env bash

RosBags="liorf_externalOdom1.bag liorf_orig.bag lvi_sam1.bag t265.bag"
roscore &
rosbag record -a -O merged.bag &

for value in $RosBags
do
   rosbag play $value & 
done