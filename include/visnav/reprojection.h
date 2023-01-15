/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <visnav/common_types.h>

namespace visnav {

template <class T>
class AbstractCamera;

struct ReprojectionCostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ReprojectionCostFunctor(const Eigen::Vector2d& p_2d,
                          const Eigen::Vector3d& p_3d,
                          const std::string& cam_model)
      : p_2d(p_2d), p_3d(p_3d), cam_model(cam_model) {}

  template <class T>
  bool operator()(T const* const sT_w_i, T const* const sT_i_c,
                  T const* const sIntr, T* sResiduals) const {
    Eigen::Map<Sophus::SE3<T> const> const T_w_i(sT_w_i);
    Eigen::Map<Sophus::SE3<T> const> const T_i_c(sT_i_c);

    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(sResiduals);
    const std::shared_ptr<AbstractCamera<T>> cam =
        AbstractCamera<T>::from_data(cam_model, sIntr);

    // TODO SHEET 2: implement the rest of the functor

    Eigen::Matrix<T, 3, 1> p_c_3d =
        (T_w_i * T_i_c).inverse().rotationMatrix() * p_3d +
        (T_w_i * T_i_c).inverse().translation();

    Eigen::Matrix<T, 2, 1> p_2d_hat = cam->project(p_c_3d);

    residuals = p_2d - p_2d_hat;

    return true;
  }

  Eigen::Vector2d p_2d;
  Eigen::Vector3d p_3d;
  std::string cam_model;
};

struct BundleAdjustmentReprojectionCostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BundleAdjustmentReprojectionCostFunctor(const Eigen::Vector2d& p_2d,
                                          const std::string& cam_model)
      : p_2d(p_2d), cam_model(cam_model) {}

  template <class T>
  bool operator()(T const* const sT_w_c, T const* const sp_3d_w,
                  T const* const sIntr, T* sResiduals) const {
    // map inputs
    Eigen::Map<Sophus::SE3<T> const> const T_w_c(sT_w_c);
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const p_3d_w(sp_3d_w);
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(sResiduals);
    const std::shared_ptr<AbstractCamera<T>> cam =
        AbstractCamera<T>::from_data(cam_model, sIntr);

    // TODO SHEET 4: Compute reprojection error
    Eigen::Matrix<T, 2, 1> p_2d_w = cam->project(T_w_c.inverse() * p_3d_w);
    residuals = p_2d - p_2d_w;
    return true;
  }

  Eigen::Vector2d p_2d;
  std::string cam_model;
};

// Make the cost functor for the IMU
struct BundleAdjustmentImuCostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BundleAdjustmentImuCostFunctor(
      const visnav::IntegratedImuMeasurement<double>& imu_meas)
      : imu_meas(imu_meas) {}

  template <class T>
  bool operator()(T const* const g, T const* const state0_T_w_i,
                  T const* const state0_v_w_i, T const* const state0_t_ns,
                  T const* const state1_T_w_i, T const* const state1_v_w_i,
                  T const* const state1_t_ns, T const* const bg,
                  T const* const ba, T* sResiduals) const {
    // Map inputs
    Eigen::Map<T const> const g(g);
    Eigen::Map<Sophus::SE3<T> const> const state0_T_w_i(state0_T_w_i);
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const state0_v_w_i(state0_v_w_i);
    T const state0_t_ns(state0_t_ns);
    Eigen::Map<Sophus::SE3<T> const> const state1_T_w_i(state1_T_w_i);
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const state1_v_w_i(state1_v_w_i);
    T const state1_t_ns(state1_t_ns);
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> bg;
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> ba;
    Eigen::Map<visnav::PoseVelState<T>::VecN> residuals(sResiduals);

    // Rebuild the state variables
    visnav::PoseVelState<T> state0;
    state0.T_w_i = state0_T_w_i;
    state0.vel_w_i = state0_v_w_i;
    state0.t_ns = state0_t_ns;
    visnav::PoseVelState<T> state1;
    state1.T_w_i = state1_T_w_i;
    state1.vel_w_i = state1_v_w_i;
    state1.t_ns = state1_t_ns;

    // Compute the residuals
    residuals = imu_meas.residual(state0, g, state1, bg, ba);
    return true;
  }
  visnav::IntegratedImuMeasurement<double> imu_meas;
};
}  // namespace visnav
