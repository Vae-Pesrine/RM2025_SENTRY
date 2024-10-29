#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <include/localization/unscented_kalman_filter.hpp>

namespace sentry_localization{

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem{
public :
    using Vector3t = Eigen::Matrix<float, 3, 1>;
    using Vector4t = Eigen::Matrix<float, 4, 1>;
    using VectorXt = Eigen::Matrix<float, Eigen::Dynamic, 1>;
    using Quaterniont = Eigen::Quaternion<float>;
public :
    PoseSystem() { dt = 0.01; }
    ~PoseSystem() {}

    VectorXt f(const VectorXt& state) const{
        VectorXt next_state(16);
        Vector3t pt = state.middleRows(0, 3);
        Vector3t vt = state.middleRows(3, 3);
        
        Quaterniont qt(state[6], state[7], state[8], state[9]);
        qt.normalize();
        Quaterniont qt_ = qt;

        Vector3t ba = state.middleRows(10, 3);
        Vector3t bg = state.middleRows(13, 3);

        next_state.middleRows(0, 3) = pt + vt * dt;
        next_state.middleRows(3, 3) = vt;
        next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
        next_state.middleRows(10, 3) = state.middleRows(10, 3);
        next_state.middleRows(13, 3) = state.middleRows(13, 3);
        
        return next_state;
    }

    VectorXt f(const VectorXt& state, const VectorXt&  control) const{
        VectorXt next_state(16);
        Vector3t pt = state.middleRows(0, 3);
        Vector3t vt = state.middleRows(3, 3);
        Quaterniont qt(state[6], state[7], state[8], state[9]);
        qt.normalize();

        Vector3t ba = state.middleRows(10, 3);
        Vector3t bg = state.middleRows(13, 3);

        Vector3t ra = control.middleRows(0, 3);
        Vector3t rg = control.middleRows(3, 3);

        next_state.middleRows(0, 3) = pt + vt * dt;

        //velocity
        Vector3t g(0.0f, 0.0f, 9.80665f);
        Vector3t acc_ = ra - ba;
        Vector3t acc = qt * acc_;
        next_state.middleRows(3, 3) = vt + (acc - g) * dt;

        // orientation
        Vector3t gyro = rg - bg;
        Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
        dq.normalize();
        Quaterniont qt_ = (qt * dq).normalized();

        next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
        next_state.middleRows(10, 3) = state.middleRows(10, 3);
        next_state.middleRows(13, 3) = state.middleRows(13, 3);
        return next_state;
    }

      // observation equation
    VectorXt h(const VectorXt& state) const {
        VectorXt observation(7);
        observation.middleRows(0, 3) = state.middleRows(0, 3);
        observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();
        return observation;
    }
    
    double dt;
};

}



#endif