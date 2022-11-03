# cctag_ros
ROS wrapper for CCTag

## Getting started (ROS noetic, ubuntu 20.04)
1. `git clone --recurse-submodules git@github.com:scottnothing/cctag_ros.git`
2. `cd cctag_ros; catkin build --this`

If that fails, you may need to edit `CCTag/CMakeLists.txt` and change the TBB version to whatever your system version is (mine is 2020.1)

`rosrun cctag_ros cctag_ros <args>` (see examples from the original repo)

## What next?
There is no ROS functionality just yet. Next step is to turn the cctag_ros.cpp into a ROS node that listens for images.
