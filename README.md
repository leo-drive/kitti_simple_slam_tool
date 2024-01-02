# Description

This package creates a map using the ground truth data in the [Kitti dataset](https://www.cvlibs.net/datasets/kitti/).
It matches the times of the incoming point cloud and the ground truth path (Using Approximate time). 
Since exact time is not used (these datas are not sync), some shifts occur when the point cloud is positioned according to ground truth. 
To correct these shifts, it registers and corrects each point cloud using NDT before adding it to the global map. 
In this way, a global point cloud is obtained with a simple slam application.

# Usage

Launch the kitti_simple_slam_tool.launch.xml file.
```bash
ros2 launch kitti_simple_slam_tool kitti_simple_slam_tool.launch.xml
```
```bash
ros2 bag play < your bag file >
```
# Parameters
| Name                | Type   | Description                          |
|---------------------|--------|--------------------------------------|
| `NDTTransEpsilon`          | double | NDT Transformation Epsilon           |
| `NDTStepSize`           | double | NDT Step Size                        |
| `NDTResolution`    | double | NDT Voxel Grid Resolution            |
| `save_pc_dir`  | string | Directory where pc map will be saved |

<br/>

#### Converting KITTI Dataset to Rosbag file:
You can convert KITTI dataset to rosbag file by using [lidar_slam_evaluator]( https://github.com/meliketanrikulu/lidar_slam_evaluator/tree/add-covariance-values
) tool. You can add covariances values to your bag file using `add-covariance-values` branch.
When you use this tool, ground truth and data bag files will be created separately.
You need to merge these bag files before using this tool. You can use [ros tools](https://github.com/ros2/rosbag2?tab=readme-ov-file#converting-bags) for merging two bag file.