// Copyright (c) 2025 Ichiro ITS
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

#ifndef GANKENKUN__WALKING__NODE__WALKING_MANAGER_HPP_
#define GANKENKUN__WALKING__NODE__WALKING_MANAGER_HPP_

#include <nlohmann/json.hpp>

#include "gankenkun/lipm/lipm.hpp"
#include "gankenkun/walking/kinematics/kinematics.hpp"
#include "gankenkun/walking/planner/foot_step_planner.hpp"

namespace gankenkun
{

class WalkingManager
{
public:
  using FootStep = FootStepPlanner::FootStep;

  WalkingManager(const std::string & path);

  void load_config(const std::string & path);
  void set_config(const nlohmann::json & walking_data, const nlohmann::json & kinematic_data);
  void update();

  const std::array<keisan::Angle<double>, 19> & get_angles() const;

  const std::list<FootStep> & set_goal_position(
    const keisan::Point2 & goal_position = keisan::Point2(-1, -1));

private:
  Kinematics kinematics;
  LIPM lipm;
  FootStepPlanner foot_step_planner;

  int status;
  int next_support;
  keisan::Angle<double> walk_orientation;

  double offset_ratio;

  double dt;

  double foot_width;
  double foot_height;
  double left_up;
  double right_up;

  keisan::Matrix<1, 3> left_offset = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> left_offset_g = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> left_offset_d = keisan::Matrix<1, 3>::zero();

  keisan::Matrix<1, 3> right_offset = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> right_offset_g = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> right_offset_d = keisan::Matrix<1, 3>::zero();
};

}  // namespace gankenkun

#endif  // GANKENKUN__WALKING__NODE__WALKING_MANAGER_HPP_
