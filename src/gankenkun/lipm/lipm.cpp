// Copyright (c) 2025 ICHIRO ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "gankenkun/lipm/lipm.hpp"

#include "gankenkun/walking/planner/foot_step_planner.hpp"
#include "keisan/problem/problem.hpp"
#include "keisan/problem/variable.hpp"
#include "keisan/problem/integrator.hpp"

#include <Eigen/Dense>

namespace gankenkun
{

LIPM::LIPM() : dt(0.0), period(0.0), z(0.0) {}

void LIPM::set_parameters(
  double z, double dt, double period, keisan::Point2 zmp_limit, int preview_steps)
{
  this->z = z;
  this->dt = dt;
  this->period = period;
  this->zmp_limit = zmp_limit;
  this->horizon = preview_steps;

  initialize();
}

// Initialize the discrete-time LTI system matrices
void LIPM::initialize()
{
  // Initialize outputs
  x_state = keisan::Matrix<3, 1>::zero();
  y_state = keisan::Matrix<3, 1>::zero();
}

// Update the LIPM state
void LIPM::update(double time, const std::deque<FootStepPlanner::FootStep> & foot_steps, bool reset)
{
  if (com_trajectory.empty() || reset) {
    x_state = keisan::Matrix<3, 1>(foot_steps.front().position.x, 0, 0);
    y_state = keisan::Matrix<3, 1>(foot_steps.front().position.y, 0, 0);
  } else {
    x_state = com_trajectory.front().x_state;
    y_state = com_trajectory.front().y_state;
  }

  com_trajectory.clear();

  auto problem = keisan::Problem();

  auto x_var = &problem.add_variable(horizon);
  auto y_var = &problem.add_variable(horizon);

  auto x = keisan::Integrator(
    *x_var, Eigen::VectorXd(Eigen::Vector3d(x_state[0][0], x_state[1][0], x_state[2][0])), 3, dt);
  auto y = keisan::Integrator(
    *y_var, Eigen::VectorXd(Eigen::Vector3d(y_state[0][0], y_state[1][0], y_state[2][0])), 3, dt);

  int foot_step_index = 0;
  std::vector<Eigen::Vector2d> current_support_polygon;

  for (int i = 0; i < horizon; ++i) {
    if (time >= foot_steps[foot_step_index].time) {
      current_support_polygon.clear();

      if (foot_steps[foot_step_index].support_foot != FootStepPlanner::BOTH_FEET) {
      }
    }
  }
}

// Pop the front of the COM trajectory
LIPM::COMTrajectory LIPM::pop_front()
{
  auto com = com_trajectory.front();
  com_trajectory.pop_front();

  return com;
}

}  // namespace gankenkun
