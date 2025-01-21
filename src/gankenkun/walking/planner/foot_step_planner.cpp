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

#include "gankenkun/walking/planner/foot_step_planner.hpp"

namespace gankenkun
{

FootStepPlanner::FootStepPlanner(const std::string & path) { load_configuration(path); }

void FootStepPlanner::load_configuration(const std::string & path)
{
  // Load the configuration from the specified path
}

std::vector<FootStepPlanner::FootStep> FootStepPlanner::plan(
  const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
  const keisan::Point2 & current_position, const keisan::Angle<double> & current_orientation,
  int next_support, int status)
{
  // Calculate the number of foot step
  double time = 0.0;
  double goal_distance = (target_position - current_position).magnitude();

  double steps_x = std::abs((target_position.x - current_position.x) / max_stride.x);
  double steps_y = std::abs((target_position.y - current_position.y) / max_stride.y);
  double steps_angle =
    std::abs(((target_orientation.degree - current_orientation).degree()) / max_rotation.degree());
  int max_steps = std::max(std::max(steps_x, steps_y), steps_angle);

  double stride_x = (target_position.x - current_position.x) / max_steps;
  double stride_y = (target_position.y - current_position.y) / max_steps;
  double stride_angle = (target_orientation.degree - current_orientation.degree()) / max_steps;

  // Plan first foot step
  std::vector<FootStep> foot_steps;
  if (status == FootStepPlanner::START) {
    foot_steps.push_back({0.0, current_position, current_orientation, FootStepPlanner::BOTH_FEET});
    time += period * 2;
  }

  if (next_support == FootStepPlanner::LEFT_FOOT) {
    foot_steps.push_back(
      {time, keisan::Point2(current_position.x, current_position.y + width), current_orientation,
       FootStepPlanner::LEFT_FOOT});
    next_support = FootStepPlanner::RIGHT_FOOT;
  } else {
    foot_steps.push_back(
      {time, keisan::Point2(current_position.x, current_position.y - width), current_orientation,
       FootStepPlanner::RIGHT_FOOT});
    next_support = FootStepPlanner::LEFT_FOOT;
  }

  // Plan walking foot steps
  size_t counter = 0;
  while (true) {
    counter += 1;

    auto delta_position = target_position - current_position;
    auto delta_orientation = target_orientation - current_orientation;

    if (
      delta_position.x < max_stride.x && delta_position.y < max_stride.y &&
      delta_orientation.degree() < max_rotation.degree()) {
      break;
    }

    time += period;

    auto next_position = current_position + keisan::Point2(stride_x, stride_y);
    auto next_orientation = current_orientation + keisan::Angle<double>(stride_angle);

    if (next_support == FootStepPlanner::LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(next_position.x, next_position.y + width), next_orientation,
         FootStepPlanner::LEFT_FOOT});
      next_support = FootStepPlanner::RIGHT_FOOT;
    } else {
      foot_steps.push_back(
        {time, keisan::Point2(next_position.x, next_position.y - width), next_orientation,
         FootStepPlanner::RIGHT_FOOT});
      next_support = FootStepPlanner::LEFT_FOOT;
    }

    current_position = next_position;
    current_orientation = next_orientation;
  }

  // Planning walk in position
  if (status != FootStepPlanner::STOP) {
    time += period;

    if (next_support == FOOT_STEP_PLANNER::LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(target_position.x, target_position.y + width), target_orientation,
         FootStepPlanner::LEFT_FOOT});
    } else {
      foot_steps.push_back(
        {time, keisan::Point2(target_position.x, target_position.y - width), target_orientation,
         FootStepPlanner::RIGHT_FOOT});
    }

    time += period;
    next_support = FootStepPlanner::BOTH_FEET;
    foot_step.push_back({time, target_position, target_orientation, FootStepPlanner::BOTH_FEET});

    time += 2.0 * period;
    foot_steps.push_back({time, target_position, target_orientation, FootStepPlanner::BOTH_FEET});

    time += 100.0;
    foot_steps.push_back({time, target_position, target_orientation, FootStepPlanner::BOTH_FEET});
  }

  return foot_steps;
}

}  // namespace gankenkun
