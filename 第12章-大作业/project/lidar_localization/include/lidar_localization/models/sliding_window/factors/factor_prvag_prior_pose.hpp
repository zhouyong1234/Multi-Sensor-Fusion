/*
* @Description: ceres residual block for gnss pose measurement
*/


#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_PRIOR_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_PRIOR_POSE_HPP_


#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGGNSSPose : public ceres::SizedCostFunction<3, 15>
{
public:
    static const int INDEX_P = 0;

    FactorPRVAGGNSSPose(void) {};
    
    void SetMeasurement(const Eigen::Vector3d &m)
    {
        m_ = m;
    }

    void SetInformation(const Eigen::Matrix3d &I)
    {
        I_ = I;
    }

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

private:
    Eigen::Vector3d m_;
    Eigen::Matrix3d I_;
};

}

#endif