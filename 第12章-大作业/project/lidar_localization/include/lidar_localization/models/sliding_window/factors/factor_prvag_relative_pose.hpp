/*
 * @Description: ceres residual block for lidar frontend relative pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGRelativePose : public ceres::SizedCostFunction<6, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;

  static const int INDEX_V = 6;
  static const int INDEX_A = 9;
  static const int INDEX_G = 12;

  FactorPRVAGRelativePose(void) {};

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
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);

    //
    // parse measurement:
    //
		const Eigen::Vector3d     &pos_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d         ori_ij = Sophus::SO3d::exp(log_ori_ij);

    // TODO: get square root of information matrix:
    // Cholesky 分解 : http://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    Eigen::LLT< Eigen::Matrix<double, 6, 6> > LowerI(I_);
    // sqrt_info 为上三角阵
    Eigen::Matrix<double, 6, 6> sqrt_info = LowerI.matrixL().transpose();

    //
    // TODO: compute residual:
    // Eigen::Matrix<double, 6, 1> resid;
    // resid.setZero();
    Eigen::Map<Eigen::Matrix<double, 6, 1> > resid(residuals);
    const Eigen::Matrix3d oriRT_i = ori_i.inverse().matrix();

    resid.block<3, 1>(INDEX_P, 0) = oriRT_i * (pos_j - pos_i) - pos_ij;
    resid.block<3, 1>(INDEX_R, 0) = ( ori_i.inverse() * ori_j * ori_ij.inverse() ).log();
    // residuals[0] = resid;

    //
    // TODO: compute jacobians:
    if ( jacobians ) {
      // compute shared intermediate results:
      Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> jacobian_i(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> jacobian_j(jacobians[1]);
      jacobian_i.setZero();
      jacobian_j.setZero();

      const Eigen::Vector3d deltaR = resid.block<3, 1>(INDEX_R, 0);
      const Eigen::Matrix3d J_r_inv = JacobianRInv(deltaR);


      if ( jacobians[0] ) {
        jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -oriRT_i;
        // jacobian_i.block<3, 3>(INDEX_P, INDEX_R) =  oriRT_i * Sophus::SO3d::hat(pos_j - pos_i);
        Eigen::Vector3d pos_ji = ori_i.inverse() * (pos_j - pos_i);
        jacobian_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(pos_ji).matrix();

        jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * (ori_ij * ori_j.inverse() * ori_i ).matrix();
      }

      if ( jacobians[1] ) {
        jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = oriRT_i;
        jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv * ori_ij.matrix();
      }

      //
      // TODO: correct residuals by square root of information matrix:
      jacobian_i = sqrt_info * jacobian_i;
      jacobian_j = sqrt_info * jacobian_j;

    }

    resid = sqrt_info * resid;

    return true;
  }

  void CheckJacobian(double **parameters)
  {
    double *residuals = new double[6];
    double **jacobians = new double *[2];
    jacobians[0] = new double[6 * 15];
    jacobians[1] = new double[6 * 15];
    Evaluate(parameters, residuals, jacobians);

    std::cout << "my jacobian: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor> >(jacobians[0]) << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor> >(jacobians[1]) << std::endl;
    std::cout << "my residual: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1> >(residuals).transpose() << std::endl;

    // a. pose i
    Eigen::Map<const Eigen::Vector3d> pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d ori_i = Sophus::SO3d::exp(log_ori_i);
    Eigen::Map<const Eigen::Vector3d> vel_i(&parameters[0][INDEX_V]);
    Eigen::Map<const Eigen::Vector3d> b_a_i(&parameters[0][INDEX_A]);
    Eigen::Map<const Eigen::Vector3d> b_g_i(&parameters[0][INDEX_G]);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d> pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d ori_j = Sophus::SO3d::exp(log_ori_j);
    Eigen::Map<const Eigen::Vector3d> vel_j(&parameters[1][INDEX_V]);
    Eigen::Map<const Eigen::Vector3d> b_a_j(&parameters[1][INDEX_A]);
    Eigen::Map<const Eigen::Vector3d> b_g_j(&parameters[1][INDEX_G]);

    const Eigen::Vector3d &pos_ij = m_.block<3, 1>(INDEX_P, 0);
    const Eigen::Vector3d &log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d ori_ij = Sophus::SO3d::exp(log_ori_ij);

    Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::LLT<Eigen::Matrix<double, 6, 6> >(I_).matrixL().transpose();

    Eigen::Matrix<double, 6, 1> residual;

    residual.block<3, 1>(INDEX_P, 0) = ori_i.inverse() * (pos_j - pos_i) - pos_ij;
    residual.block<3, 1>(INDEX_R, 0) = (ori_i.inverse() * ori_j * ori_ij.inverse()).log();
    residual = sqrt_information * residual;

    std::cout << "residual: " << std::endl;
    std::cout << residual.transpose() << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 6, 30> num_jacobian;
    for (int k = 0; k < 30; ++k)
    {
      // pose i
      Eigen::Vector3d pos_i_num = pos_i;
      Sophus::SO3d ori_i_num = ori_i;
      Eigen::Vector3d vel_i_num = vel_i;
      Eigen::Vector3d b_a_i_num = b_a_i;
      Eigen::Vector3d b_g_i_num = b_g_i;
      // pose j
      Eigen::Vector3d pos_j_num = pos_j;
      Sophus::SO3d ori_j_num = ori_j;
      Eigen::Vector3d vel_j_num = vel_j;
      Eigen::Vector3d b_a_j_num = b_a_j;
      Eigen::Vector3d b_g_j_num = b_g_j;

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0)
        pos_i_num = pos_i_num + delta;
      else if (a == 1)
        ori_i_num = ori_i_num * Sophus::SO3d::exp(delta);
      else if (a == 2)
        vel_i_num = vel_i_num + delta;
      else if (a == 3)
        b_a_i_num = b_a_i_num + delta;
      else if (a == 4)
        b_g_i_num = b_g_i_num + delta;
      else if (a == 5)
        pos_j_num = pos_j_num + delta;
      else if (a == 6)
        ori_j_num = ori_j_num * Sophus::SO3d::exp(delta);
      else if (a == 7)
        vel_j_num = vel_j_num + delta;
      else if (a == 8)
        b_a_j_num = b_a_j_num + delta;
      else if (a == 9)
        b_g_j_num = b_g_j_num + delta;

      Eigen::Matrix<double, 6, 1> tmp_residual;
      tmp_residual.block<3, 1>(INDEX_P, 0) = ori_i_num.inverse() * (pos_j_num - pos_i_num) - pos_ij;
      tmp_residual.block<3, 1>(INDEX_R, 0) = (ori_i_num.inverse() * ori_j_num * ori_ij.inverse()).log();
      tmp_residual = sqrt_information * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    std::cout << "num jacobian:" << std::endl;
    std::cout << num_jacobian.block<6, 15>(0, 0) << std::endl;
    std::cout << num_jacobian.block<6, 15>(0, 15) << std::endl
              << std::endl;
  }

private:
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

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
