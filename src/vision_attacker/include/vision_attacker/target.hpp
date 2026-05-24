#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include "auto_aim_interfaces/msg/target.hpp"

namespace auto_aim
{

class Target
{
public:
  Target() = default;

  // Fill Target from tracker message that already contains EKF-filtered state.
  // This method directly sets the internal state vector used by planner.
  void from_msg(const auto_aim_interfaces::msg::Target & msg,
                std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now());

  // Return internal state vector used by planner (compatible with previous ekf_x())
  Eigen::VectorXd ekf_x() const;

  // Predict forward by dt seconds using a simple kinematic model (used by planner)
  void predict(double dt);
  void predict(std::chrono::steady_clock::time_point t);

  // Return list of armors as (x,y,z,yaw) for planner usage
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool isinit = false;

private:
  Eigen::VectorXd state_ = Eigen::VectorXd::Zero(11);
  int armor_num_ = 4;
  std::chrono::steady_clock::time_point t_ = std::chrono::steady_clock::now();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP