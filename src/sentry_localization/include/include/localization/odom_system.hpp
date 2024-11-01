#ifndef ODOM_SYSTEM_HPP
#define ODOM_SYSTEM_HPP

#include <include/localization/unscented_kalman_filter.hpp>

namespace sentry_localization {
    
/**
 * @brief This class models the sensor pose estimation based on robot odometry
 * @note state = [px, py, pz, qw, qx, qy, qz]
 *       observation = [px, py, pz, qw, qx, qy, qz]
 *       maybe better to use expmap
 */
class OdomSystem {
public:
  using Vector3t = Eigen::Matrix<float, 3, 1>;
  using Vector4t = Eigen::Matrix<float, 4, 1>;
  using Matrix4t = Eigen::Matrix<float, 4, 4>;
  using VectorXt = Eigen::Matrix<float, Eigen::Dynamic, 1>;
  using Quaterniont = Eigen::Quaternion<float>;

public:

  // system equation
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
      Matrix4t pt = Matrix4t::Identity();
      pt.block<3, 1>(0, 3) = Vector3t(state[0], state[1], state[2]);
      pt.block<3, 1>(0, 3) = Vector3t(state[0], state[1], state[2]);
      pt.block<3, 3>(0, 0) = Quaterniont(state[3], state[4], state[5], state[6]).normalized().toRotationMatrix();

      Matrix4t delta = Matrix4t::Identity();
      delta.block<3, 1>(0, 3) = Vector3t(control[0], control[1], control[2]);
      delta.block<3, 3>(0, 0) = Quaterniont(control[3], control[4], control[5], control[6]).normalized().toRotationMatrix();
  
      Matrix4t pt_ = pt * delta;
      Quaterniont quat_(pt_.block<3, 3>(0, 0));
  
      VectorXt next_state(7);
      next_state.head<3>() = pt_.block<3, 1>(0, 3);
      next_state.tail<4>() = Vector4t(quat_.w(), quat_.x(), quat_.y(), quat_.z());
  
      return next_state;
  }

  // observation equation
  VectorXt h(const VectorXt& state) const {
    return state;
  }

};

}

#endif 
