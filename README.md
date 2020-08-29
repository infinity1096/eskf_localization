# eskf_localization

## Description
ROS implementation of the Error State Kalman Filter according to *Quaternion kinematics for the error-state Kalman filter* by Joan Solà. 
Full text can be obtained [here](https://arxiv.org/abs/1711.02508).

Node fuses information from IMU, GPS, and magnetometer, publishes fused pose.

For more details, consult: https://zhuanlan.zhihu.com/c_1266722926339645440

## Usage
```
roslaunch eskf_localization eskf_localization.launch
```
## Reference
- https://zhuanlan.zhihu.com/p/152662055
- dataset: https://epan-utbm.github.io/utbm_robocar_dataset/
  - recommended: https://drive.utbm.fr/s/MR7DdRcWYysk9Do/download
