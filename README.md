# eskf_localization

![Fused Path](https://github.com/infinity1096/eskf_localization/blob/master/ros_wrapper/pictures/rviz_screenshot_2020_08_29-19_44_14.png)

## Description
ROS implementation of the Error State Kalman Filter according to *Quaternion kinematics for the error-state Kalman filter* by Joan Solà. 
Full text can be obtained [here](https://arxiv.org/abs/1711.02508).

Node fuses information from IMU, GPS, and magnetometer, publishes fused pose.

For more details, consult: https://zhuanlan.zhihu.com/c_1266722926339645440

## Usage
```
roslaunch eskf_localization eskf_localization.launch
```
On another terminal,
```
rosbag play utbm_robocar_dataset_20180719_noimage.bag
```

## Reference
- https://zhuanlan.zhihu.com/p/152662055
- dataset: https://epan-utbm.github.io/utbm_robocar_dataset/
  - recommended: https://drive.utbm.fr/s/MR7DdRcWYysk9Do/download
