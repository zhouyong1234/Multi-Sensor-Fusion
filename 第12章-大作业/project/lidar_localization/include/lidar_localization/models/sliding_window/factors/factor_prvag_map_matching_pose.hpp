/*
 * @Description: ceres residual block for map matching pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGMapMatchingPose : public ceres::SizedCostFunction<6, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;

  FactorPRVAGMapMatchingPose(void) {};

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // pose
    Eigen::Map<const Eigen::Vector3d>     pos(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori = Sophus::SO3d::exp(log_ori);

    //
    // parse measurement:
    //
		const Eigen::Vector3d     &pos_prior = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &log_ori_prior = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d         ori_prior = Sophus::SO3d::exp(log_ori_prior);

    // TODO: get square root of information matrix:
    // Cholesky 分解 : http://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    Eigen::LLT< Eigen::Matrix<double, 6, 6> > LowerI(I_);
    // sqrt_info 为上三角阵
    Eigen::Matrix<double, 6, 6> sqrt_info = LowerI.matrixL().transpose();

    //
    // TODO: compute residual:
    // Eigen::Matrix<double, 6, 1> resid;
    // resid.setZero();
    // 地图匹配位姿和优化变量的残差
    Eigen::Map<Eigen::Matrix<double, 6, 1> > resid(residuals);
    resid.block<3, 1>(INDEX_P, 0) = pos - pos_prior;
    resid.block<3, 1>(INDEX_R, 0) = (ori * ori_prior.inverse()).log();
    // residuals[0] = resid;

    //
    // TODO: compute jacobians:
    //
    if ( jacobians ) {
      Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_(jacobians[0]);
      jacobian_.setZero();

      const Eigen::Vector3d deltaR = resid.block<3, 1>(INDEX_R, 0);
      const Eigen::Matrix3d J_r_inv = JacobianRInv(deltaR);

      if ( jacobians[0] ) {
        // implement jacobian computing:
        jacobian_.block<3, 3>(INDEX_P, INDEX_P) = Eigen::Matrix3d::Identity();
        jacobian_.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv * ori_prior.matrix();
      }

      //
      // TODO: correct residual by square root of information matrix:
      jacobian_ = sqrt_info * jacobian_;
    }

    resid = sqrt_info * resid;

    return true;
  }

private:
  // static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w)
  // {
  //   Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

  //   double theta = w.norm();

  //   if (theta > 1e-5)
  //   {
  //     Eigen::Vector3d k = w.normalized();
  //     Eigen::Matrix3d K = Sophus::SO3d::hat(k);

  //     J_r_inv = J_r_inv + 0.5 * K + (1.0  - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
  //   }

  //   return J_r_inv;
  // }
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w)
  {
    Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

    double theta = w.norm();

    if (theta > 1e-5)
    {
      Eigen::Vector3d a = w.normalized();
      Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
      double theta_half = 0.5 * theta;
      double cot_theta = 1.0 / tan(theta_half);

      J_r_inv = theta_half * cot_theta * J_r_inv + (1.0 - theta_half * cot_theta) * a * a.transpose() + theta_half * a_hat;
    }

    return J_r_inv;
  }

  static Eigen::Matrix3d JacobianR(const Eigen::Vector3d &w)
  {
    Eigen::Matrix3d J_r = Eigen::Matrix3d::Identity();

    double theta = w.norm();

    if (theta > 1e-5)
    {
      Eigen::Vector3d a = w.normalized();
      Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);

      J_r = sin(theta) / theta * Eigen::Matrix3d::Identity() + (1.0 - sin(theta) / theta) * a * a.transpose() - (1.0 - cos(theta)) / theta * a_hat;
    }

    return J_r;
  }
  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_HPP_
