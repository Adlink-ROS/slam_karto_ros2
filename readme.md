# slam karto (ROS 2.0)  
This is ROS 2.0 version pkg for slam_karto.   
The original repo (ROS 1.0) was developed by OSRF team.  

The original version needs dependency: open_karto    
However, in order to decrease the complicility of compilation in ament env,  
we mergered open_karto into slam_karto for ROS 2.0 version.  

## Developer  
* HaoChih, LIN (haochih.lin@adlinktech.com)  

## License  
LGPL License (adhere to the original slam_karto & open_karto pkg)  
  
## Dependency
* sparse_bundle_adjustment (ROS 2.0 version)  
$ git clone https://github.com/Adlink-ROS/sparse_bundle_adjustment_ros2  
$ ament build --isolated --build-tests --symlink-install --only sparse_bundle_adjustment  
  
## Compile       
$ cd ~/ros2_ws  
$ ament build --only-packages slam_karto  
  
For isolated build  
$ ament build --isolated --symlink-install --only slam_karto  

## Execute  
$ ros2 run slam_karto slam_karto  
  
## Visualization
Currently, rviz2 doesn't support "nav_msgs/MapMetaData", hence, you can use ros1_bridge to see the result in ROS 1.0 rviz.  
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics  

## Roadmap   
* Add parameter service  
* Add use_sim_time support  
