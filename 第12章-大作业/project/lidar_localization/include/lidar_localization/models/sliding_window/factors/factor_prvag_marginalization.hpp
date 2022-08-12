/*
 * @Description: ceres residual block for sliding window marginalization
 * @Author: Ge Yao
 * @Date: 2021-01-05 21:57:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGMarginalization : public ceres::SizedCostFunction<15, 15> {
public:
	static const int INDEX_M =  0;
  static const int INDEX_R = 15;

  FactorPRVAGMarginalization(void) {
    H_ = Eigen::MatrixXd::Zero(30, 30);
    b_ = Eigen::VectorXd::Zero(30);

    J_ = Eigen::MatrixXd::Zero(15, 15);
    e_ = Eigen::VectorXd::Zero(15);
  }

  void SetResMapMatchingPose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    // init:
    ResidualBlockInfo res_map_matching_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    // compute:
    // 调用的是factor_prvag_map_matching_pose.hpp的Evaluate
    Evaluate(res_map_matching_pose, residuals, jacobians);
    const Eigen::MatrixXd &J_m = jacobians.at(0);

    //
    // TODO: Update H:
    // a. H_mm:
    const Eigen::MatrixXd H_mm = J_m.transpose() * J_m;
    H_.block<15, 15>(INDEX_M, INDEX_M) += H_mm;

    //
    // TODO: Update b:
    // a. b_m:
    const Eigen::MatrixXd b_m = J_m.transpose() * residuals;
    b_.block<15, 1>(INDEX_M, 0) += b_m;
  }

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

  void SetResRelativePose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    // init:
    ResidualBlockInfo res_relative_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    // compute:
    // 调用的是factor_prvag_relative_pose.hpp的Evaluate
    Evaluate(res_relative_pose, residuals, jacobians);
    const Eigen::MatrixXd &J_m = jacobians.at(0);
    const Eigen::MatrixXd &J_r = jacobians.at(1);

    //
    // TODO: Update H:
    // 注意顺序
    // a. H_mm:
    const Eigen::MatrixXd H_mm = J_m.transpose() * J_m;
    H_.block<15, 15>(INDEX_M, INDEX_M) += H_mm;
    // b. H_mr:
    const Eigen::MatrixXd H_mr = J_m.transpose() * J_r;
    H_.block<15, 15>(INDEX_M, INDEX_R) += H_mr;
    // c. H_rm:
    const Eigen::MatrixXd H_rm = J_r.transpose() * J_m;
    H_.block<15, 15>(INDEX_R, INDEX_M) += H_rm;
    // d. H_rr:
    const Eigen::MatrixXd H_rr = J_r.transpose() * J_r;
    H_.block<15, 15>(INDEX_R, INDEX_R) += H_rr;

    //
    // TODO: Update b:
    // a. b_m:
    const Eigen::MatrixXd b_m = J_m.transpose() * residuals;
    b_.block<15, 1>(INDEX_M, 0) += b_m;
    // a. b_r:
    const Eigen::MatrixXd b_r = J_r.transpose() * residuals;
    b_.block<15, 1>(INDEX_R, 0) += b_r;
  }

  void SetResIMUPreIntegration(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    // init:
    ResidualBlockInfo res_imu_pre_integration(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    // compute:
    // 调用的是factor_prvag_imu_pre_integration.hppv的Evaluate
    Evaluate(res_imu_pre_integration, residuals, jacobians);
    const Eigen::MatrixXd &J_m = jacobians.at(0);
    const Eigen::MatrixXd &J_r = jacobians.at(1);

    //
    // TODO: Update H:
    // 注意顺序
    // a. H_mm:
    const Eigen::MatrixXd H_mm = J_m.transpose() * J_m;
    H_.block<15, 15>(INDEX_M, INDEX_M) += H_mm;
    // b. H_mr:
    const Eigen::MatrixXd H_mr = J_m.transpose() * J_r;
    H_.block<15, 15>(INDEX_M, INDEX_R) += H_mr;
    // c. H_rm:
    const Eigen::MatrixXd H_rm = J_r.transpose() * J_m;
    H_.block<15, 15>(INDEX_R, INDEX_M) += H_rm;
    // d. H_rr:
    const Eigen::MatrixXd H_rr = J_r.transpose() * J_r;
    H_.block<15, 15>(INDEX_R, INDEX_R) += H_rr;

    //
    // TODO: Update b:
    // a. b_m:
    const Eigen::MatrixXd b_m = J_m.transpose() * residuals;
    b_.block<15, 1>(INDEX_M, 0) += b_m;
    // a. b_r:
    const Eigen::MatrixXd b_r = J_r.transpose() * residuals;
    b_.block<15, 1>(INDEX_R, 0) += b_r;

  }

  void Marginalize(
    const double *raw_param_r_0
  ) {
    // TODO: implement marginalization logic
    //save x_m_0;
    Eigen::Map<const Eigen::Matrix<double, 15, 1> > x_raw(raw_param_r_0);
    // 原始线性化点
    x_0_ = x_raw;
    //marginalize
    const Eigen::MatrixXd H_mm_ = H_.block<15, 15>(INDEX_M, INDEX_M);
    const Eigen::MatrixXd H_mr_ = H_.block<15, 15>(INDEX_M, INDEX_R);
    const Eigen::MatrixXd H_rm_ = H_.block<15, 15>(INDEX_R, INDEX_M);
    const Eigen::MatrixXd H_rr_ = H_.block<15, 15>(INDEX_R, INDEX_R);
    const Eigen::VectorXd b_m_  = b_.block<15,  1>(INDEX_M, 0);
    const Eigen::VectorXd b_r_  = b_.block<15,  1>(INDEX_R, 0);

    Eigen::MatrixXd H_tmp = H_rr_ - H_rm_ * ( H_mm_.inverse() ) * H_mr_;
    Eigen::MatrixXd b_tmp = b_r_ - H_rm_ * ( H_mm_.inverse() ) * b_m_;
    // case1 : 直接求解 failed

    // case2 : SelfAdjointEigenSolver,特征分解
    // H_tmp 是对称矩阵，所以是hermitian矩阵，
    // 代码中使用Eigen::SelfAdjointEigenSolver完成对herrmitian的SVD分解。得到:
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H_tmp);
    // 这部分参考vins代码: https://zhuanlan.zhihu.com/p/51330624；(但是公式好像有点问题，思路是差不多的)
    // H = J.t() * J = V * S * V.t() = ( S.sqrt() * V.t() ).t() * ( S.sqrt() * V.t() )
    //   >> J = S.sqrt() * V.t()
    // b = J.t() * f
    //   >> f = ( J.t() )^{-1} * b = V * S.sqrt() * b

    // (R.array() > s).select(P,Q)  -> (R > s ? P : Q)
    Eigen::VectorXd S = Eigen::VectorXd((saes.eigenvalues().array() > 1.0e-5).select(saes.eigenvalues().array(), 0));
    // eigenvalues().array().inverse() ：这一步没有找到资料，但是猜测是对每个元素求逆，即求倒数
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > 1.0e-5).select(saes.eigenvalues().array().inverse(), 0));

    // cwiseSqrt : sqrt()
    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt(); // S^{-1/2}
    // 从边缘化后的信息矩阵中恢复出来雅克比矩阵linearized_jacobians和残差linearized_residuals，
    // 这两者会作为先验残差带入到下一轮的先验残差的雅克比和残差的计算当中去
    // J = S^{1/2} * V.t()
    J_ = S_sqrt.asDiagonal() * saes.eigenvectors().transpose();
    // e_0 = (V * S^{-1/2}).t() * b
    e_ = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * b_tmp;
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    Eigen::Map<const Eigen::Matrix<double, 15, 1>> x(parameters[0]);
    Eigen::VectorXd dx = x - x_0_;

    //
    // TODO: compute residual:
    Eigen::Map< Eigen::Matrix<double, 15, 1> > residual(residuals);
    // TODO:
    // 先验残差的更新：可以使用一阶泰勒近似
    residual = e_ + J_ * dx;

    //
    // TODO: compute jacobian:
    if ( jacobians ) {
      if ( jacobians[0] ) {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor> > jacobian_(jacobians[0]);
        jacobian_.setZero();
        jacobian_ = J_;
      }
    }

    return true;
  }

private:
  Eigen::MatrixXd H_;
  Eigen::VectorXd b_;

  Eigen::MatrixXd J_;
  Eigen::VectorXd e_;

  Eigen::VectorXd x_0_;

  struct ResidualBlockInfo {
    const ceres::CostFunction *residual = nullptr;
    std::vector<double *> parameter_blocks;

    ResidualBlockInfo(void) {}

    ResidualBlockInfo(
      const ceres::CostFunction *_residual,
      const std::vector<double *> &_parameter_blocks
    ) : residual(_residual), parameter_blocks(_parameter_blocks) {}
  };

  static void Evaluate(
    ResidualBlockInfo &residual_info,
    Eigen::VectorXd &residuals,
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> &jacobians
  ) {
    // init residual output:
    const int D = static_cast<int>(residual_info.residual->num_residuals());
    residuals.resize(D);

    // init jacobians output:
    std::vector<int> block_sizes = residual_info.residual->parameter_block_sizes();
    const int N = static_cast<int>(block_sizes.size());

    double **raw_jacobians = new double *[N];
    jacobians.resize(N);

    // create raw pointer adaptor:
    for (int i = 0; i < N; i++) {
      jacobians[i].resize(D, block_sizes[i]);
      raw_jacobians[i] = jacobians[i].data();
    }

    residual_info.residual->Evaluate(
      residual_info.parameter_blocks.data(),
      residuals.data(), raw_jacobians
    );
  }
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
