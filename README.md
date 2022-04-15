# LiDAR-LANE-DETECTION

Simple algorithm to detect the lane of road environment using 3D LiDAR <br/>

<br/>

## Tested Environment

- Ubuntu 18.04
- ROS Melodic

<br/>

## Summary

* Subscribe `sensor_msgs/PointCloud2` msg
* Lane Detection from the point cloud
* Publish lane points: the pink color one

![](/lidar-lane.gif)

<br/>

## How to use

Clone, build and run

~~~bash
$ git clone https://github.com/bigbigpark/LiDAR-LANE-DETECTION.git
$ git clone https://github.com/bigbigpark/lidar_msgs.git
~~~

~~~bash
$ catkin build
~~~

~~~bash
$ roslaunch lidar_lane_detection lane_detect.launch
~~~

<br/>

## Parameter configuration

You can easily modify topic name of `sensor_msgs/PointCloud2` by changing **lane_detect.launch** <br/>

~~~xml
<launch>
  <node name="lane_detection" pkg="lidar_lane_detection" type="lane_detection" respawn="true" output="screen">
    <remap from="/input" to="/os_cloud_node/points"/>
  </node>

  <arg name="rviz_config" default="$(find lidar_lane_detection)/lane.rviz"/>
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rviz_config)" required="true" />
  
</launch>
~~~

Here, change your topic <br/>

<br/>

## TODO

- [ ] Detection of more far lanes
