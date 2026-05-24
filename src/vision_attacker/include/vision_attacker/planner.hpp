#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <vector>

#include "vision_attacker/target.hpp"
#include "tinympc/tiny_api.hpp"
#include "vision_attacker/trajectory.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;

  Plan()
    : control(false), fire(false), target_yaw(0.0f), target_pitch(0.0f), yaw(0.0f), yaw_vel(0.0f), yaw_acc(0.0f), pitch(0.0f), pitch_vel(0.0f), pitch_acc(0.0f)
  {
  }
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  
  Planner(
    double yaw_offset,
    double pitch_offset ,
    double fire_thresh , 
    double low_speed_delay_time , 
    double high_speed_delay_time , 
    double decision_speed,

    double max_yaw_acc,
    double max_pitch_acc,
    std::vector<double> Q_yaw_v,
    std::vector<double> R_yaw_v,
    std::vector<double> Q_pitch_v,
    std::vector<double> R_pitch_v,
    double lag_time, double air_k
  );

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(double max_yaw_acc_ , std::vector<double> Q_yaw_ , std::vector<double> R_yaw_);
  void setup_pitch_solver(double max_pitch_acc_ , std::vector<double> Q_pitch_ , std::vector<double> R_pitch_);

  double lag_time_;
  double air_k_;
  double J_yaw_ = 1.0;
  double J_pitch_ = 1.0;


  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP