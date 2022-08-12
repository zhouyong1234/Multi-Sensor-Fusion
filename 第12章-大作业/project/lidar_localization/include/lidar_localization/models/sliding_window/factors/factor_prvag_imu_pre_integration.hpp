/*
 * @Description: ceres residual block for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGIMUPreIntegration : public ceres::SizedCostFunction<15, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;
	static const int INDEX_V = 6;
	static const int INDEX_A = 9;
	static const int INDEX_G = 12;

  FactorPRVAGIMUPreIntegration(void) {};

	void SetT(const double &T) {
		T_ = T;
	}

	void SetGravitiy(const Eigen::Vector3d &g) {
		g_ = g;
	}

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

	void SetJacobian(const Eigen::MatrixXd &J) {
		J_ = J;
	}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);
		Eigen::Map<const Eigen::Vector3d>     vel_i(&parameters[0][INDEX_V]);
		Eigen::Map<const Eigen::Vector3d>     b_a_i(&parameters[0][INDEX_A]);
		Eigen::Map<const Eigen::Vector3d>     b_g_i(&parameters[0][INDEX_G]);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);
		Eigen::Map<const Eigen::Vector3d>     vel_j(&parameters[1][INDEX_V]);
		Eigen::Map<const Eigen::Vector3d>     b_a_j(&parameters[1][INDEX_A]);
		Eigen::Map<const Eigen::Vector3d>     b_g_j(&parameters[1][INDEX_G]);

    // parse measurement:
		Eigen::Vector3d alpha_ij = m_.block<3, 1>(INDEX_P, 0);
		Eigen::Vector3d theta_ij = m_.block<3, 1>(INDEX_R, 0);
		Eigen::Vector3d  beta_ij = m_.block<3, 1>(INDEX_V, 0);

    // bias变化量
    Eigen::Vector3d dba = b_a_i - m_.block<3, 1>(INDEX_A, 0);
    Eigen::Vector3d dbg = b_g_i - m_.block<3, 1>(INDEX_G, 0);

    // 根据变化量使用jacobian重新计算预积分
    Eigen::Matrix3d J_q_bg = J_.block<3, 3>(INDEX_R, INDEX_G);
    Eigen::Matrix3d J_v_ba = J_.block<3, 3>(INDEX_V, INDEX_A);
    Eigen::Matrix3d J_v_bg = J_.block<3, 3>(INDEX_V, INDEX_G);
    Eigen::Matrix3d J_p_ba = J_.block<3, 3>(INDEX_P, INDEX_A);
    Eigen::Matrix3d J_p_bg = J_.block<3, 3>(INDEX_P, INDEX_G);

    Sophus::SO3d ori_ij = Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_q_bg * dbg);

    alpha_ij = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
    beta_ij = beta_ij + J_v_ba * dba + J_v_bg * dbg;

    //
    // TODO: get square root of information matrix:
    // Cholesky 分解 : http://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    Eigen::LLT<Eigen::Matrix<double,15,15>> LowerI(I_);
    // sqrt_info 为上三角阵
    Eigen::Matrix<double,15,15> sqrt_info = LowerI.matrixL().transpose();

    //
    // TODO: compute residual:
    // 有向量计算，转换为矩阵再进行(但是可以不转换，即 ori_i.inverse() == ori_i.matrix().transpose())
    // 全是向量计算，不用再转换为矩阵了
    const Eigen::Matrix3d oriRT_i = ori_i.inverse().matrix();
    Eigen::Map<Eigen::Matrix<double, 15, 1> > resid(residuals);
    resid.block<3, 1>(INDEX_P, 0) = oriRT_i * (pos_j - pos_i - vel_i * T_ + 0.5 * g_ * T_ * T_) - alpha_ij;
    resid.block<3, 1>(INDEX_R, 0) = (ori_ij.inverse() * ( ori_i.inverse() * ori_j ) ).log();
    resid.block<3, 1>(INDEX_V, 0) = oriRT_i * ( vel_j - vel_i + g_ * T_) - beta_ij;
    resid.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
    resid.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;
    // resid.block<3, 1>(INDEX_P, 0) = ori_i.inverse() * (pos_j - pos_i - vel_i * T_ + 0.5 * g_ * T_ * T_) - alpha_ij;
    // resid.block<3, 1>(INDEX_R, 0) = (ori_ij.inverse() * ori_i.inverse() * ori_j).log();
    // resid.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
    // resid.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
    // resid.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;


    //
    // TODO: compute jacobians:
    //
    if ( jacobians ) {
      // compute shared intermediate results:
      Eigen::Map<Eigen::Matrix<double,15,15,Eigen::RowMajor>> jacobian_i(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,15,15,Eigen::RowMajor>> jacobian_j(jacobians[1]);
      jacobian_i.setZero();
      jacobian_j.setZero();

      const Eigen::Vector3d deltaR = resid.block<3, 1>(INDEX_R, 0);
      const Eigen::Matrix3d J_r_inv = JacobianRInv(deltaR);

      if ( jacobians[0] ) {
        // a. residual, position:
        jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -oriRT_i;
        jacobian_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(ori_i.inverse() * (pos_j - pos_i - vel_i * T_ + 0.50 * g_ * T_ * T_)).matrix();
        jacobian_i.block<3, 3>(INDEX_P, INDEX_V) = -oriRT_i * T_;
        jacobian_i.block<3, 3>(INDEX_P, INDEX_A) = -J_.block<3, 3>(INDEX_P,INDEX_A);
        jacobian_i.block<3, 3>(INDEX_P, INDEX_G) = -J_.block<3, 3>(INDEX_P,INDEX_G);

        // b. residual, orientation:
        jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * ((ori_j.inverse() * ori_i).matrix());
        // jacobian_i.block<3, 3>(INDEX_R, INDEX_G) = -J_r_inv * (Sophus::SO3d::exp(resid.block<3, 1>(INDEX_R, 0))).matrix().inverse() * J_.block<3, 3>(INDEX_R, INDEX_G);
        // note: b_g_i - m_.block<3, 1>(INDEX_G, 0) or b_g_i
        jacobian_i.block<3, 3>(INDEX_R, INDEX_G) \
              = -J_r_inv \
              * Sophus::SO3d::exp(resid.block<3, 1>(INDEX_R, 0)).matrix().inverse() \
              * JacobianR(J_.block<3, 3>(INDEX_R, INDEX_G) * (b_g_i - m_.block<3, 1>(INDEX_G, 0))) \
              * J_.block<3, 3>(INDEX_R, INDEX_G);

        // c. residual, velocity:
        jacobian_i.block<3, 3>(INDEX_V, INDEX_R) =  Sophus::SO3d::hat(ori_i.inverse() * (vel_j - vel_i + g_ * T_));
        jacobian_i.block<3, 3>(INDEX_V, INDEX_V) = -oriRT_i;
        jacobian_i.block<3, 3>(INDEX_V, INDEX_A) = -J_.block<3, 3>(INDEX_V, INDEX_A);
        jacobian_i.block<3, 3>(INDEX_V, INDEX_G) = -J_.block<3, 3>(INDEX_V, INDEX_G);

        // d. residual, bias accel:
        jacobian_i.block<3, 3>(INDEX_A, INDEX_A) =  -Eigen::Matrix3d::Identity();
        jacobian_i.block<3, 3>(INDEX_G, INDEX_G) =  -Eigen::Matrix3d::Identity();
      }

      if ( jacobians[1] ) {
        // a. residual, position:
        jacobian_j.block<3,3>(INDEX_P,INDEX_P) = oriRT_i;
        // b. residual, orientation:
        jacobian_j.block<3,3>(INDEX_R,INDEX_R) = J_r_inv;
        // c. residual, velocity:
        jacobian_j.block<3,3>(INDEX_V,INDEX_V) = oriRT_i;
        // d. residual, bias accel:
        jacobian_j.block<3,3>(INDEX_A,INDEX_A) = Eigen::Matrix3d::Identity();
        jacobian_j.block<3,3>(INDEX_G,INDEX_G) = Eigen::Matrix3d::Identity();
      }

      //
      // TODO: correct residual by square root of information matrix:
      jacobian_i = sqrt_info * jacobian_i;
      jacobian_j = sqrt_info * jacobian_j;
    }

    resid = sqrt_info * resid;

    return true;
  }

  void CheckJacobian(double **parameters)
  {
    double *residuals = new double[15];
    double **jacobians = new double *[2];
    jacobians[0] = new double[15 * 15];
    jacobians[1] = new double[15 * 15];
    Evaluate(parameters, residuals, jacobians);

    std::cout << "my jacobian: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor> >(jacobians[0]) << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor> >(jacobians[1]) << std::endl;
    std::cout << "my residual: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 15, 1> >(residuals).transpose() << std::endl;

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

    Eigen::Vector3d alpha_ij = m_.block<3, 1>(INDEX_P, 0);
    Eigen::Vector3d theta_ij = m_.block<3, 1>(INDEX_R, 0);
    Eigen::Vector3d beta_ij = m_.block<3, 1>(INDEX_V, 0);

    Eigen::Matrix<double, 15, 15> sqrt_information = Eigen::LLT<Eigen::Matrix<double, 15, 15> >(I_).matrixL().transpose();

    Eigen::Matrix<double, 15, 1> residual;

    residual.block<3, 1>(INDEX_P, 0) = ori_i.inverse() * (pos_j - pos_i - vel_i * T_ + 0.5 * g_ * T_ * T_) - alpha_ij;
    Sophus::SO3d ori_ij = Sophus::SO3d::exp(theta_ij);
    residual.block<3, 1>(INDEX_R, 0) = (ori_ij.inverse() * ori_i.inverse() * ori_j).log();
    residual.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
    residual.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
    residual.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;

    residual = sqrt_information * residual;

    std::cout << "residual: " << std::endl;
    std::cout << residual.transpose() << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 15, 30> num_jacobian;
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

      // bias变化量
      Eigen::Vector3d dba = b_a_i_num - b_a_i;
      Eigen::Vector3d dbg = b_g_i_num - b_g_i;
      // std::cout<<"d_ba: " << dba.transpose()<<std::endl;
      // std::cout<<"ba_i: " << b_a_i.transpose()<<std::endl;
      // std::cout<<"ba: " << m_.block<3, 1>(INDEX_A, 0).transpose()<<std::endl<<std::endl;

      // 根据变化量使用jacobian重新计算预积分
      Eigen::Matrix3d J_q_bg = J_.block<3, 3>(INDEX_R, INDEX_G);
      Eigen::Matrix3d J_v_ba = J_.block<3, 3>(INDEX_V, INDEX_A);
      Eigen::Matrix3d J_v_bg = J_.block<3, 3>(INDEX_V, INDEX_G);
      Eigen::Matrix3d J_p_ba = J_.block<3, 3>(INDEX_P, INDEX_A);
      Eigen::Matrix3d J_p_bg = J_.block<3, 3>(INDEX_P, INDEX_G);

      Eigen::Vector3d alpha_ij_turb = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
      // std::cout<<"after: "<< alpha_ij.transpose()<<std::endl<<std::endl;

      // Eigen::Vector3d theta_ij_trub = theta_ij + J_q_bg*dbg;

      Eigen::Vector3d beta_ij_turb = beta_ij + J_v_ba * dba + J_v_bg * dbg;
      // Sophus::SO3d ori_ij_trub = Sophus::SO3d::exp(theta_ij_trub);
      Sophus::SO3d ori_ij_trub = Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_q_bg * dbg);

      Eigen::Matrix<double, 15, 1> tmp_residual;

      tmp_residual.block<3, 1>(INDEX_P, 0) = ori_i_num.inverse() * (pos_j_num - pos_i_num - vel_i_num * T_ + 0.5 * g_ * T_ * T_) - alpha_ij_turb;
      tmp_residual.block<3, 1>(INDEX_R, 0) = (ori_ij_trub.inverse() * ori_i_num.inverse() * ori_j_num).log();
      tmp_residual.block<3, 1>(INDEX_V, 0) = ori_i_num.inverse() * (vel_j_num - vel_i_num + g_ * T_) - beta_ij_turb;
      tmp_residual.block<3, 1>(INDEX_A, 0) = b_a_j_num - b_a_i_num;
      tmp_residual.block<3, 1>(INDEX_G, 0) = b_g_j_num - b_g_i_num;

      tmp_residual = sqrt_information * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    std::cout << "num jacobian:" << std::endl;
    std::cout << num_jacobian.block<15, 15>(0, 0) << std::endl;
    std::cout << num_jacobian.block<15, 15>(0, 15) << std::endl
              << std::endl;
  }

private:
  // static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w)
  // {
  //       Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

  //       double theta = w.norm();

  //       if ( theta > 1e-5 ) {
  //           Eigen::Vector3d k = w.normalized();
  //           Eigen::Matrix3d K = Sophus::SO3d::hat(k);

  //           J_r_inv = J_r_inv
  //                     + 0.5 * K
  //                     + (1.0  - (1.0 + std::cos(theta)) * theta
  //                       / (2.0 * std::sin(theta))) * K * K;
  //       }

  //       return J_r_inv;
  // }

  // static Eigen::Matrix3d JacobianR(const Eigen::Vector3d &w)
  // {
  //   Eigen::Matrix3d J_r = Eigen::Matrix3d::Identity();

  //   double theta = w.norm();

  //   if (theta > 1e-5)
  //   {
  //     Eigen::Vector3d a = w.normalized();
  //     Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);

  //     J_r
  //       = Eigen::Matrix3d::Identity()
  //       - ( 1.0 - cos(theta) ) / theta  * a_hat
  //       + ( 1.0 - sin(theta) * 1.0 / theta)  * a_hat * a_hat;
  //   }
  //   return J_r;
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
    double T_ = 0.0;

    Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

    Eigen::VectorXd m_;
    Eigen::MatrixXd I_;

    Eigen::MatrixXd J_;
  };

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
