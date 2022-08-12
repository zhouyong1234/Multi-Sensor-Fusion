# 大作业

项目代码放在了project文件夹下，代码细节和测试结果在<br>
[多传感器融合定位(一) 点云地图构建和基于点云地图定位](https://zhuanlan.zhihu.com/p/537456620)
<br>
[多传感器融合定位(二) 基于滤波+运动约束的定位](https://zhuanlan.zhihu.com/p/537887234)
<br>
[多传感器融合定位(三) 基于预积分的建图](https://zhuanlan.zhihu.com/p/537979907)
<br>
[多传感器融合定位(四) 基于预积分的定位](https://zhuanlan.zhihu.com/p/538240992)
<br>
[多传感器融合定位(五) 不同定位算法绝对轨迹误差对比](https://zhuanlan.zhihu.com/p/539108494)
<br>


# 基于优化的定位，添加gnss先验边，代码在./project/lidar_localization


## src/matching/back_end/sliding_window.cpp
```
// 添加GNSS先验边
    if(N > 0 && measurement_config_.source.use_gnss)
    {
        Eigen::Vector3d gnss_pose = current_key_gnss_.pose.block<3,1>(0, 3).cast<double>();

        // Eigen::Matrix3d tmp;
        // tmp << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        // gnss_pose = tmp * gnss_pose;

        // add GNSS position
        sliding_window_ptr_->AddPRVAGPriorPoseFactor(
            param_index_j,
            gnss_pose, measurement_config_.noise.gnss_position);
    }
```

由于是一元边，只须N>0即可，GNSS提供了位置约束，坐标系已经对齐，直接加入


## src/models/sliding_window/ceres_sliding_window.cpp

#### AddPRVAGPriorPoseFactor的实现

```
void CeresSlidingWindow::AddPRVAGPriorPoseFactor(
      const int param_index,
      const Eigen::Vector3d &pose, const Eigen::Vector3d &noise
)
{
    ResidualGNSSPose residual_gnss_pose;

    residual_gnss_pose.param_index = param_index;

    residual_gnss_pose.m = Eigen::Vector3d::Zero();
    residual_gnss_pose.m.block<3,1>(INDEX_P, 0) = pose;

    residual_gnss_pose.I = GetInformationMatrix(noise);

    // add to data buffer:
    residual_blocks_.gnss_pose.push_back(residual_gnss_pose);   
}   
```

观测值为三维xyz坐标，信息矩阵为noise的倒数
```
Eigen::Matrix3d CeresSlidingWindow::GetInformationMatrix(Eigen::Vector3d noise)
{
    Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();

    for(int i = 0; i < noise.rows(); i++)
    {
        information_matrix(i, i) /= noise(i);
    }

    return information_matrix;
}
```


## 在ceres中添加gnss先验残差块

#### CeresSlidingWindow::Optimize() 

```
if(!residual_blocks_.gnss_pose.empty())
{
    const ceres::CostFunction *factor_gnss_pose = GetResGNSSPose(
        residual_blocks_.gnss_pose.front()
    );

    factor_marginalization->SetResGNSSPose(
        factor_gnss_pose,
        std::vector<double *>{key_frame_m.prvag}
    );
}
```

获取gnss先验因子
```
sliding_window::FactorPRVAGGNSSPose *CeresSlidingWindow::GetResGNSSPose(
    const CeresSlidingWindow::ResidualGNSSPose &res_gnss_pose
)
{
    sliding_window::FactorPRVAGGNSSPose *factor_gnss_pose = new sliding_window::FactorPRVAGGNSSPose();

    factor_gnss_pose->SetMeasurement(res_gnss_pose.m);
    factor_gnss_pose->SetInformation(res_gnss_pose.I);

    return factor_gnss_pose;
}
```

#### FactorPRVAGGNSSPose在 lidar_localization/models/sliding_window/factors/factor_prvag_prior_pose.hpp

```
virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        // pose
        Eigen::Map<const Eigen::Vector3d> pose(&parameters[0][INDEX_P]);

        // measurement
        const Eigen::Vector3d &pose_prior = m_.block<3,1>(INDEX_P, 0);

        Eigen::LLT< Eigen::Matrix<double, 3, 3> > LowerI(I_);

        Eigen::Matrix<double, 3, 3> sqrt_info = LowerI.matrixL().transpose();

        // GNSS先验位置和优化变量的残差
        Eigen::Map<Eigen::Matrix<double, 3, 1> > resid(residuals);
        resid.block<3,1>(INDEX_P, 0) = pose - pose_prior;

        // std::cout << "resid: " << resid.transpose() << std::endl;

        // 计算雅可比
        if(jacobians)
        {
            Eigen::Map<Eigen::Matrix<double, 3, 15, Eigen::RowMajor>> jacobian_(jacobians[0]);
            jacobian_.setZero();

            if(jacobians[0])
            {
                jacobian_.block<3,3>(INDEX_P, INDEX_P) = Eigen::Matrix3d::Identity();
            }

            jacobian_ = sqrt_info * jacobian_;
        }

        resid = sqrt_info * resid;

        return true;
    }
```

残差的维度为3x1,雅可比的维度为3x15,注意jacobian_.setZero()，不然会报错

#### lidar_localization/models/sliding_window/factors/factor_prvag_marginalization.hpp边缘化因子中添加和GNSS先验边有关的矩阵块

```
void SetResGNSSPose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  )
  {
    ResidualBlockInfo res_gnss_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;


    // 调用的是factor_prvag_prior_pose.hpp的Evaluate
    Evaluate(res_gnss_pose, residuals, jacobians);

    // Update H
    const Eigen::MatrixXd &J_m = jacobians.at(0);
    const Eigen::MatrixXd H_mm = J_m.transpose() * J_m;
    H_.block<15, 15>(INDEX_M, INDEX_M) += H_mm;

    // Update b
    const Eigen::MatrixXd b_m = J_m.transpose() * residuals;
    b_.block<15, 1>(INDEX_M, 0) += b_m;

  }
```

H矩阵维度为30x30，和前后两帧位姿有关，GNSS只约束一帧位姿，因此添加的H矩阵块为15x15, b矩阵块为15x1


#### 执行边缘化操作后，需要移除对应的因子

```
// 边缘化
factor_marginalization->Marginalize(key_frame_r.prvag);

// add marginalization factor into sliding window
problem.AddResidualBlock(
    factor_marginalization,
    NULL,
    key_frame_r.prvag
);


residual_blocks_.map_matching_pose.pop_front();
residual_blocks_.relative_pose.pop_front();
residual_blocks_.imu_pre_integration.pop_front();

if(!residual_blocks_.gnss_pose.empty())
{
    residual_blocks_.gnss_pose.pop_front();
}
```

#### 在ceres中添加GNSS残差块
src/models/sliding_window/ceres_sliding_window.cpp
```
// GNSS prior constraint
       
if(!residual_blocks_.gnss_pose.empty())
{
    for(const auto &residual_gnss_pose: residual_blocks_.gnss_pose)
    {
        auto &key_frame = optimized_key_frames_.at(residual_gnss_pose.param_index);

        sliding_window::FactorPRVAGGNSSPose *factor_gnss_pose = GetResGNSSPose(
            residual_gnss_pose
        );

        problem.AddResidualBlock(
            factor_gnss_pose,
            NULL,
            key_frame.prvag);
    }
}
```


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
