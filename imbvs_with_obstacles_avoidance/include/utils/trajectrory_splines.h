/// \author Dimitar Rakov

#ifndef TRAJECTORY__SPLINES_H
#define TRAJECTORY__SPLINES_H

#include <Eigen/Core>
#include <Eigen/QR>

/// Spline Trajectory
class TrajectrorySpline
{
public:
  TrajectrorySpline() : pos_s_(0), vel_s_(0), pos_e_(0), vel_e_(0),duration_(0) {}

  void calcSpline(double pos_s, double pos_e, double duration){
    pos_s_ = pos_s;
    pos_e_ = pos_e;
    duration_= duration;


    cubic_param_(0) = pos_s_;
    cubic_param_(1) = (pos_e_- pos_s_)/duration;
    cubic_param_(2) = 0.0;
    cubic_param_(3) = 0.0;
  }


  void calcSpline(double pos_s, double vel_s, double pos_e, double vel_e, double duration){
    pos_s_ = pos_s;
    vel_s_ = vel_s;
    pos_e_ = pos_e;
    vel_e_ = vel_e;
    duration_= duration;

    M_(0, 0) = 1.0;
    M_(0, 1) = 0.0;
    M_(0, 2) = 0.0;
    M_(0, 3) = 0.0;

    M_(1, 0) = 0.0;
    M_(1, 1) = 1.0;
    M_(1, 2) = 0.0;
    M_(1, 3) = 0.0;

    M_(2, 0) = 1.0;
    M_(2, 1) = duration;
    M_(2, 2) = duration * duration;
    M_(2, 3) = duration * duration *duration;

    M_(3, 0) = 0;
    M_(3, 1) = 1.0;
    M_(3, 2) = 2.0 * duration;
    M_(3, 3) = 3.0 * duration *duration;

    cubic_param_ = M_.colPivHouseholderQr().solve(Eigen::Vector4d(pos_s, vel_s, pos_e, vel_e));

   // std::cout << "\ncubic_param_" << cubic_param_.transpose();
  }
  double getPos(double time){
    if (time < 0.0)
      return pos_s_;
    else if (time > duration_)
        return pos_e_;
    else
      return cubic_param_(0) + cubic_param_(1) * time + cubic_param_(2) * time * time + cubic_param_(3) * time * time * time;

  }
  double getVel(double time){
    if (time < 0.0)
      return vel_s_;
    else if (time > duration_)
      return vel_e_;
    else
      return cubic_param_(1) + 2.0 * cubic_param_(2) * time + 3 * cubic_param_(3) * time * time;
  }


private:
  Eigen::Vector4d cubic_param_;
  Eigen::Matrix4d M_;
  double pos_s_;
  double vel_s_;
  double pos_e_;
  double vel_e_;
  double duration_;

};

#endif
