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

#include "keisan/problem/constraint.hpp"
#include "keisan/problem/expression.hpp"
#include "keisan/problem/integrator.hpp"
#include "keisan/problem/polygon_constraint.hpp"
#include "keisan/problem/problem.hpp"
#include "keisan/problem/variable.hpp"

#include <Eigen/Dense>

namespace gankenkun
{

LIPM::LIPM() : dt(0.0), period(0.0), z(0.0), zmp_limit(keisan::Point2(0.0, 0.0)), horizon(0) {}

void LIPM::set_parameters(double z, double dt, double period, keisan::Point2 zmp_limit, int horizon)
{
  this->z = z;
  this->dt = dt;
  this->period = period;
  this->zmp_limit = zmp_limit;
  this->horizon = horizon;

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
    x_state[0][0] = foot_steps.front().position.x;
    y_state[0][0] = foot_steps.front().position.y;
  } else {
    x_state = com_trajectory.front().x_state;
    y_state = com_trajectory.front().y_state;
  }

  com_trajectory.clear();

  int current_horizon =
    std::min(horizon, static_cast<int>(round((foot_steps.back().time - time) / dt)));

  double omega = (9.81 / z);
  double omega_2 = omega * omega;

  auto problem = keisan::Problem();

  keisan::Variable * x_var = &problem.add_variable(current_horizon);
  keisan::Variable * y_var = &problem.add_variable(current_horizon);

  keisan::Integrator x(
    *x_var, Eigen::VectorXd(Eigen::Vector3d(x_state[0][0], x_state[1][0], x_state[2][0])), 3, dt);
  keisan::Integrator y(
    *y_var, Eigen::VectorXd(Eigen::Vector3d(y_state[0][0], y_state[1][0], y_state[2][0])), 3, dt);

  int foot_step_index = 1;
  std::vector<Eigen::Vector2d> current_support_polygon;

  // TODO: generate based on horizon in the future
  for (int i = 1; i <= current_horizon; ++i) {
    auto zmp_expression = (x.expr(i, 0) - (1.0 / omega_2) * x.expr(i, 2)) /
                          (y.expr(i, 0) - (1.0 / omega_2) * y.expr(i, 2));

    auto support_foot = foot_steps[foot_step_index - 1];

    if (time >= foot_steps[foot_step_index].time) {
      current_support_polygon.clear();

      // In a single support phase
      if (support_foot.support_foot != FootStepPlanner::BOTH_FEET) {
        current_support_polygon.push_back(Eigen::Vector2d(
          support_foot.position.x - zmp_limit.x, support_foot.position.y + zmp_limit.y));
        current_support_polygon.push_back(Eigen::Vector2d(
          support_foot.position.x + zmp_limit.x, support_foot.position.y + zmp_limit.y));
        current_support_polygon.push_back(Eigen::Vector2d(
          support_foot.position.x + zmp_limit.x, support_foot.position.y - zmp_limit.y));
        current_support_polygon.push_back(Eigen::Vector2d(
          support_foot.position.x - zmp_limit.x, support_foot.position.y - zmp_limit.y));
      } else {  // In a double support phase
        // TODO: implement DSP
      }

      ++foot_step_index;
    }

    // Hard constraint to keep ZMP in the support polygon
    if (!current_support_polygon.empty()) {
      problem.add_constraint(
        keisan::PolygonConstraint::in_polygon(zmp_expression, current_support_polygon, 0.0));
    }

    // Soft constraint to keep ZMP in the middle of support foot
    problem
      .add_constraint(
        zmp_expression == Eigen::Vector2d(support_foot.position.x, support_foot.position.y))
      .configure(keisan::Constraint::Soft, 1.0);
  }

  problem.solve();

  for (int i = 0; i < static_cast<int>(round((foot_steps[2].time - time) / dt)); ++i) {
    auto pos = keisan::Point2(x.value(dt * i, 0), y.value(dt * i, 0));
    auto vel = keisan::Point2(x.value(dt * i, 1), y.value(dt * i, 1));
    auto acc = keisan::Point2(x.value(dt * i, 2), y.value(dt * i, 2));

    auto com = COMTrajectory();
    com.position = keisan::Point2(pos.x, pos.y);

    com.x_state[0][0] = pos.x;
    com.x_state[1][0] = vel.x;
    com.x_state[2][0] = acc.x;

    com.y_state[0][0] = pos.y;
    com.y_state[1][0] = vel.y;
    com.y_state[2][0] = acc.y;

    std::cout << "current horizon : " << current_horizon << "/"
              << static_cast<int>(round((foot_steps.back().time - time) / dt)) << std::endl;
    std::cout << "horizon to next step : " << static_cast<int>(round((foot_steps[2].time - time) / dt)) << std::endl;
    std::cout << "x state : " << pos.x << ", " << vel.x << ", " << acc.x << std::endl;
    std::cout << "y state : " << pos.y << ", " << vel.y << ", " << acc.y << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    com_trajectory.push_back(com);
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
