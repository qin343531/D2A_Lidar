这个是我买了个19块钱的激光雷达,我从商家给的SDK源码修改为ROS2架构的,可以发布/scan话题,但是我还不知道怎么建图,会在终端打印四个方向信息
![](goods.png)
# 构建项目
```
cd
mkdir ros2_ws && cd ros2_ws
mkdir src && cd src
git clone https://github.com/qin343531/D2A_Lidar.git
cd ../
colcon build
```
# 运行脚本
记得给你的串口设备权限,例如`sudo chmod 666 /dev/ttyUSB0`
```
source install/setup.bash
ros2 run lidar_ros2 lidar_publisher 
```
