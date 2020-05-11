# ROS package to accomplish planar model-fitting.

This package is to accomplish planar model-fitting given a triangle or square shaped planar point cloud.  

## Dependencies
1. Eigen3
2. Ceres Solver
3. Sophus

## Content
1. Model fitting (sensor_msgs::PointCloud2)

## 1. Model fitting (sensor_msgs::PointCloud2)

### (a) Setup
Before we start running the model-fitting module, there is one configuration file needed to be specfied.  
We use ROS Parameter Server to set the configurations.  

1. path:  
Their path are  
```
~/model_fitting/config/lidar_config.yaml
```

2. format:  
**lidar_config.yaml** are used for launching **LidarProcessor_node**.  

**lidar_config.yaml**  
> topic_name_lidar: "/points_raw"  
> box_filter_set:  
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
**box_filter_set** is used to extract target points from pointcloud by predifining a center and a cube scope.  
**intensity_filter_set** is used to extract target points from pointcloud by predifining a intensity region.  
**target_size_set** is the specification of target ("triangle" or "square"). (unit: m)  
**lidar_resolution_set** is the specification of LiDAR resolution (unit: deg).  


### (b) Getting Started.
1. Launch the node  
```
roslaunch model_fitting lidar.launch
```

2. Overlook into the X-Y plane, observe and use **rosparam set** to specify the possible location of target.  
For instance,
```
rosparam set /LidarProcessor_node/passthrough_filter_set/center [7,5,0]
```
Or set the location in **lidar_config.yaml** directly.  

3. Play the bag, then start processing.  
After bag is finished, one output file **lidar_data_raw.csv** would be generated in ```~/model_fitting/data/```. It contains the target center and the vertices with timestamp:  
(**List order**: time_stamp, c_x, c_y, c_z, v1_x, v1_y, v1_z, ..., c_xx, c_yy, czz)   
* Notice that (c_x, c_y, c_y) is the center of the triangle (square) model, and (c_xx, c_yy, c_zz) is the center of the corner reflector.  


