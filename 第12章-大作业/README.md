# Multi-Sensor Fusion for Localization & Mapping -- 多传感器融合定位与建图: Capstone Project

深蓝学院多传感器融合定位与建图Capstone Project思路提示.

---

## Overview

本提示旨在引导您:

* 成功完成Capstone Project

---

## Introduction

各位同学好, 今天为大家带来大作业的思路分享.

---

## Dataset

首先, 我们用到的数据集是 [Click Here](https://github.com/weisongwen/UrbanNavDataset). 数据集中总共有三个场景的数据, 分别是:

* 两个香港的场景
* 和一个东京的场景

**需要注意的是, 大家在写作业的时候不要使用东京的数据**. 数据集的作者伟松老师说, 目前因为疫情原因, 东京的数据一直没更新, 所以还没有能使用的外参. 所以大家只需要测试前两个数据就可以, 不需要在第三个场景上进行测试了.

---

## Sensor Adaptation

此处以第四章的作业为例, 主要修改的地方是**data_pretreat模块**. 

### Sensor Freq. Change

**KITTI**的数据中，GPS和IMU的数据频率都是**10hz**. 而**urbannav**中:

* **GPS**是1hz
* **Lidar Pointcloud**是10hz
* **IMU**是100hz

与之对应, 需要修改的文件如下:

* **data_pretreat_flow.cpp**中函数**ValidData()**中的**0.05, 应选用点云数据周期的一半, 这里不需要更改.
* **gnss_data.cpp**和**imu_data.cpp**中数据同步函数**SyncData**中的参数, 同样地, 应该根据传感器的数据发布周期进行修改.

### Topic Pub-Sub Change

数据集中的Topic Name亦发生了变化. 与之对应， 需要修改的Topic如下:

* **Lidar** 从/kitti/velo/pointcloud, 改为/velodyne_points
* **IMU** 从/kitti/oxts/imu, 改为/imu/data
* **GPS** 从/kitti/oxts/gps/fix, 改为/navsat/fix

### Frame Change

同时还需要修改外参. 集邮代码的外参是通过订阅TF来实现的:

```c++
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
``` 

因此在代码中, 还需要要修改这一部分. 

### GPS Acquisition

若需解析novatel_data的数据, 请安装对应的msg:

```bash
sudo apt-get install ros-kinetic-novatel-msgs
```

**注意**:不要使用第一个bag中的/ublox_node/fix，这个是普通的单点定位，误差在几米内

### Speed Acquisition

至于速度话题, 新的bag文件中没有, 可以通过:

* 雷达里程计获得速度估计;
* 对相邻两帧的GPS数据差分获得速度估计;

### Gravity Change

此外, 还需要修改使用的重力加速度常数. [Click Here](https://www.sensorsone.com/local-gravity-calculator) 计算香港对应的g值, 并在对应的配置文件中进行修改:

```yaml
error_state_kalman_filter:
    gravity_magnitude: to-be-updated
    latitude: to-be-updated
```

### Best-Practice

推荐各位单独创建一个topic和外参的yaml, 这样方便测试其他的数据集或者直接在真车上测试. 外参具体数据可以去看 [Dataset Github](UrbanNavDataset/UrbanNav-HK-Data20200314/extrinsic.yaml) 

---

## Wrap-Up

好了，大作业思路分享到此结束，祝大家学习愉快, 成功全优结业, 收获理想的Offer!
