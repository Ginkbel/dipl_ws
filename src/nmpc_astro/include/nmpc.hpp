#ifndef NMPC_H
#define NMPC_H

#include <casadi/casadi.hpp>
#include <iostream>
#include <algorithm>

// Astro constraints
#define MAX_LINEAR_VELOCITY 0.25   // m s-1
#define MAX_ANGULAR_VELOCITY 1.0  // rad s-1
#define MAX_DELTA_VELOCITY 0.1   // m s-2
#define MAX_DELTA_OMEGA M_PI/8.0   // rad s-1


namespace nmpc_controller
{

class NMPCController {
    public:
      NMPCController();
      void setUp();
      std::pair<std::vector<double>, casadi::DM> solve(const std::vector<double> x0);
      void setReference(const std::vector<double> x_ref);
      void setReferenceTrajectoryFigure8(double normalized_time);
      void setReferenceTrajectoryCircular(double normalized_time);
    private:
      casadi::Opti opti_;
      int T_; // horizon
      int nx_; // state size
      int nu_; // control size
      double dt_; // sampling time
      casadi::Dict solver_options_;
      casadi::MX Q;
      casadi::MX R;
      casadi::MX P;
      casadi::MX p_;
      casadi::MX x_;
      casadi::MX _x_ref;
      casadi::MX _u_ref;
      casadi::DM x_init;
      casadi::MX u_;
      casadi::DM u_init;
      casadi::MX J_;
      casadi::MX kinematics(const casadi::MX& x, const casadi::MX& u, const double dt);
    
  };
}

#endif

float unwrap(float previous_angle, float new_angle);