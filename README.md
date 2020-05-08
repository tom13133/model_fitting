# ROS package to accomplish target center estimation.

This package is to estimate the target center from ***Velodyne LiDAR*** messages.  

## Dependencies
1. Eigen3
2. Ceres Solver
3. Sophus

## Content
1. Target center from Velodyne LiDAR (sensor_msgs::PointCloud2)

## 1. Target center from Velodyne LiDAR (sensor_msgs::PointCloud2)

### (a) Setup
Before we start running the estimation module, there is one configuration files needed to be specfied.  
We use ROS Parameter Server to set the configurations.  

1. path:  
Their path are  
```
~/target_processing/config/lidar_config.yaml
```

2. format:  
**lidar_config.yaml** are used for launching **LidarProcessor_node**.  

**lidar_config.yaml**  
> topic_name_lidar: "/points_raw"  
> passthrough_filter_set:  
>   center: [0, 0, 0]  
>   cube_side_length: [2,2,10]  
> intensity_filter_set:  
>   lower_upper_bound: [0, 150]  
> target_size_set:  
>   model_type: triangle  
>   center_to_end_length: 0.6  
>   reflector_edge_length: 0.282843  
> lidar_resolution_set:  
>   vertical_resolution: 1.33  
>   horizontal_resolution: 0.16  

**topic_name_lidar** specify the topic name for subsciber.  
**passthrough_filter_set** is used to extract target points from pointcloud by predifining a center and a cube scope.  
**intensity_filter_set** is used to extract target points from pointcloud by predifining a intensity region.  
**target_size_set** is the specification of target ("triangle" or "square"). (unit: m)  
**lidar_resolution_set** is the specification of Velodyne LiDAR resolution (unit: deg).  


### (b) Getting Started.
1. Launch the node  
```
roslaunch target_processing lidar.launch
```

2. Overlook into the X-Y plane, observe and use **rosparam set** to specify the possible location of target.  
For instance,
```
rosparam set /LidarProcessor_node/passthrough_filter_set/center [7,5,0]
```
Or set the location in **lidar_config.yaml** directly.  

3. Play the bag, then start processing.  
After bag is finished, one output file **lidar_data_raw.csv** would be generated in ```~/target_processing/data/```. It contains the target center and the vertices with timestamp:  
(**List order**: time_stamp, c_x1, c_y1, c_z1, c_x2, c_y2, c_z2, vertices, ...)   
* Notice that (c_x1, c_y1, c_y2) is the center of the corner reflector, and (c_x2, c_y2, c_z2) is the center of the triangle board  


