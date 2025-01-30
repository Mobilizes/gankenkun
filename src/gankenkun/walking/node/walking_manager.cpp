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

#include "gankenkun/walking/node/walking_manager.hpp"

using namespace keisan::literals;

namespace gankenkun
{

WalkingManager::WalkingManager(const std::string & path) { load_config(path); }

void WalkingManager::WalkingManager(const std::string & path)
{
  // TODO: Load config
}

const std::list<FootStep> & set_goal(
  const keisan::Point2 & goal_position = keisan::Point2(-1, -1),
  const keisan::Angle<double> & goal_orientation)
{
  if (goal_position.x == -1 && goal_position.y == -1) {
    if (foot_step_planner.foot_steps <= 4) {
      status = FootStepPlanner::START;
    }

    if (foot_step_planner.foot_steps.size() > 3) {
      foot_step_planner.foot_steps.pop_front();
    }
  } else {
    keisan::Point2 current_position = keisan::Point2(0.0, 0.0);
    keisan::Angle<double> current_orientation = 0.0_deg;

    if (foot_step_planner.foot_steps.size() > 2) {
      double y_offset = 0.0;

      if (status != FootStepPlanner::START) {
        y_offset = next_support == FootStepPlanner::LEFT_FOOT ? -0.06 : 0.06;  // TODO: Config this
      }

      current_position.x = foot_step_planner.foot_steps[1].position.x;
      current_position.y = foot_step_planner.foot_steps[1].position.y + y_offset;
      current_orientation = foot_step_planner.foot_steps[1].orientation;
    }

    foot_step_planner.plan(
      goal_position, goal_orientation, current_position, current_orientation, next_support, status);

    status = FootStepPlanner::WALKING;
  }

  double time = foot_step_planner.foot_steps[0].time;
  lipm.update(time, foot_step_planner.foot_steps);

  if (foot_step_planner.foot_steps[0].support_foot == FootStepPlanner::LEFT_FOOT) {
    if (foot_step_planner.foot_steps[1].support_foot == FootStepPlanner::BOTH_FEET) {
      right_offset_g = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[1].position.x, foot_step_planner.foot_steps[1].position.y,
        foot_step_planner.foot_steps[1].orientation);
    } else {
      right_offset_g = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[1].position.x,
        foot_step_planner.foot_steps[1].position.y + 0.06,
        foot_step_planner.foot_steps[1].orientation);  // TODO: Config this
    }

    right_offset_d = (right_offset_g - right_offset) / offset_ratio;
    next_support = FootStepPlanner::RIGHT_FOOT;
  } else if (foot_step_planner.foot_steps[0].support_foot == FootStepPlanner::RIGHT_FOOT) {
    if (foot_step_planner.foot_steps[1].support_foot == FootStepPlanner::BOTH_FEET) {
      left_offset_g = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[1].position.x, foot_step_planner.foot_steps[1].position.y,
        foot_step_planner.foot_steps[1].orientation);
    } else {
      left_offset_g = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[1].position.x,
        foot_step_planner.foot_steps[1].position.y - 0.06,
        foot_step_planner.foot_steps[1].orientation);  // TODO: Config this
    }

    left_offset_d = (left_offset_g - left_offset) / offset_ratio;
    next_support = FootStepPlanner::LEFT_FOOT;
  }

  walk_orientation = foot_step_planner.foot_steps[0].orientation;

  return foot_step_planner.foot_steps;
}

void WalkingManager::update()
{
  auto com = lipm.get_com_trajectory().front();

  double period =
    round(foot_step_planner.foot_steps[1].time - foot_step_planner.foot_steps[0].time / dt);

  walk_orientation +=
    (foot_step_planner.foot_steps[1].orientation - foot_step_planner.foot_steps[0].orientation) /
    keisan::make_degree(period);

  double dsp_ratio = round(dsp_duration / dt);
  double start_up = round(dsp_ratio / 2);
  double end_up = round(period / 2);
  double period_up = end_up - start_up;

  if (foot_step_planner.foot_steps[0].support_foot == FootStepPlanner::LEFT_FOOT) {
    // Raise or lower right foot
    double diff = period - lipm.get_trajectory().size();
    if (start_up < diff && diff <= end_up) {
      right += foot_height / period_up;
    } else if (right > 0.0) {
      right = std::max(right - foot_height / period_up, 0.0);
    }

    // Move foot to target position and orientation
    if (diff > start_up) {
      right += right_d;

      if (diff > (start_up + period_up * 2)) {
        right = right_g;
      }
    }
  } else if (foot_step_planner.foot_steps[0].support_foot == FootStepPlanner::RIGHT_FOOT) {
    // Raise or lower left foot
    double diff = period - lipm.get_trajectory().size();
    if (start_up < diff && diff <= end_up) {
      left_up += foot_height / period_up;
    } else if (left_up > 0.0) {
      left_up = std::max(left_up - foot_height / period_up, 0.0);
    }

    // Move foot to target position and orientation
    if (diff > start_up) {
      left_offset += left_offset_d;

      if (diff > (start_up + period_up * 2)) {
        left_offset = left_offset_g;
      }
    }
  }

  auto left_off = keisan::Matrix<1, 3>(
    left_offset[0][0] - com.position.x, left_offset[0][1] - com.position.y, left_offset[0][2]);

  auto right_off = keisan::Matrix<1, 3>(
    right_offset[0][0] - com.position.x, right_offset[0][1] - com.position.y, right_offset[0][2]);

  Kinematics::Foot left_foot;
  left_foot.position.x = left_off[0][0];
  left_foot.position.y = left_off[0][1] + foot_width / 2;
  left_foot.position.z = left_up;
  left_foot.yaw = walk_orientation - keisan::make_radian(left_off[0][2]);

  Kinematics::Foot right_foot;
  right_foot.position.x = right_off[0][0];
  right_foot.position.y = right_off[0][1] - foot_width / 2;
  right_foot.position.z = right_up;
  right_foot.yaw = walk_orientation - keisan::make_radian(right_off[0][2]);

  kinematics.update(left_foot, right_foot);
}

const std::array<keisan::Angle<double>, 19> & WalkingManager::get_angles() const
{
  return kinematics.get_angles();
}

}  // namespace gankenkun
