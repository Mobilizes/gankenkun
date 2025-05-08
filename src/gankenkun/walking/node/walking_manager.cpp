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

#include <fstream>

#include "jitsuyo/config.hpp"

using namespace keisan::literals;

namespace gankenkun
{

WalkingManager::WalkingManager()
: initialized(false),
  left_up(0.0),
  right_up(0.0),
  robot_position(keisan::Point2(0.0, 0.0)),
  robot_orientation(0.0_deg),
  time_step(0.008),
  status(FootStepPlanner::START),
  next_support(FootStepPlanner::RIGHT_FOOT),
  dsp_duration(0.0),
  plan_period(0.0),
  step_frames(0.0),
  com_height(0.0),
  foot_height(0.0),
  feet_lateral(0.0),
  foot_offset(keisan::Point3(0.0, 0.0, 0.0)),
  step_y_offset(0.0),
  odometry_offset(keisan::Point2(0.0, 0.0)),
  max_stride(keisan::Point2(0.0, 0.0)),
  max_rotation(0.0_deg)
{
  using tachimawari::joint::Joint;
  using tachimawari::joint::JointId;

  for (auto id : JointId::list) {
    joints.push_back(Joint(id, 0.0));
  }
}

void WalkingManager::load_config(const std::string & path)
{
  std::ifstream walking_file(path + "walking.json");
  nlohmann::json walking_data = nlohmann::json::parse(walking_file);

  std::ifstream kinematic_file(path + "kinematic.json");
  nlohmann::json kinematic_data = nlohmann::json::parse(kinematic_file);

  set_config(walking_data, kinematic_data);

  walking_file.close();
  kinematic_file.close();

  set_goal(keisan::Point2(0.0, 0.0), 0.0_deg);
}

void WalkingManager::set_config(
  const nlohmann::json & walking_data, const nlohmann::json & kinematic_data)
{
  bool valid_config = true;

  nlohmann::json timing_section;
  if (jitsuyo::assign_val(walking_data, "timing", timing_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(timing_section, "dsp_duration", dsp_duration);
    valid_section &= jitsuyo::assign_val(timing_section, "plan_period", plan_period);
    valid_section &= jitsuyo::assign_val(timing_section, "com_period", com_period);
    valid_section &= jitsuyo::assign_val(timing_section, "step_frames", step_frames);

    if (!valid_section) {
      std::cout << "Error found at section `timing`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json posture_section;
  if (jitsuyo::assign_val(walking_data, "posture", posture_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(posture_section, "com_height", com_height);
    valid_section &= jitsuyo::assign_val(posture_section, "foot_height", foot_height);
    valid_section &= jitsuyo::assign_val(posture_section, "feet_lateral", feet_lateral);

    if (!valid_section) {
      std::cout << "Error found at section `posture`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json offset_section;
  if (jitsuyo::assign_val(walking_data, "offset", offset_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(offset_section, "foot_x_offset", foot_offset.x);
    valid_section &= jitsuyo::assign_val(offset_section, "foot_y_offset", foot_offset.y);
    valid_section &= jitsuyo::assign_val(offset_section, "foot_z_offset", foot_offset.z);
    valid_section &= jitsuyo::assign_val(offset_section, "step_y_offset", step_y_offset);
    valid_section &= jitsuyo::assign_val(offset_section, "odometry_x_offset", odometry_offset.x);
    valid_section &= jitsuyo::assign_val(offset_section, "odometry_y_offset", odometry_offset.y);

    if (!valid_section) {
      std::cout << "Error found at section `offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json stride_section;
  if (jitsuyo::assign_val(walking_data, "stride", stride_section)) {
    bool valid_section = true;

    double max_rotation_double;

    valid_section &= jitsuyo::assign_val(stride_section, "max_x", max_stride.x);
    valid_section &= jitsuyo::assign_val(stride_section, "max_y", max_stride.y);
    valid_section &= jitsuyo::assign_val(stride_section, "max_a", max_rotation_double);

    max_rotation = keisan::make_degree(max_rotation_double);

    if (!valid_section) {
      std::cout << "Error found at section `stride`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json com_section;
  if (jitsuyo::assign_val(walking_data, "com", com_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(com_section, "zmp_limit_x", zmp_limit.x);
    valid_section &= jitsuyo::assign_val(com_section, "zmp_limit_y", zmp_limit.y);
    valid_section &= jitsuyo::assign_val(com_section, "horizon", horizon);

    if (!valid_section) {
      std::cout << "Error found at section `com`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `walking.json`");
  }

  foot_step_planner.set_parameters(max_stride, max_rotation, plan_period, step_y_offset);

  lipm.set_parameters(com_height, time_step, com_period, zmp_limit, horizon);

  kinematics.set_config(kinematic_data);
}

void WalkingManager::set_position(const keisan::Point2 & position) { robot_position = position; }

void WalkingManager::set_orientation(const keisan::Angle<double> & orientation)
{
  robot_orientation = orientation;
}

bool WalkingManager::is_running() { return status == FootStepPlanner::WALKING; }

void WalkingManager::stop() { set_goal(robot_position, robot_orientation); }

void WalkingManager::remove_steps()
{
  if (foot_step_planner.foot_steps.size() <= 5) {
    status = FootStepPlanner::START;
  }

  if (foot_step_planner.foot_steps.size() > 4) {
    foot_step_planner.foot_steps.pop_front();
  }
}

bool WalkingManager::replan() { return lipm.get_com_trajectory().empty(); }

void WalkingManager::set_goal(
  const keisan::Point2 & goal_position, const keisan::Angle<double> & goal_orientation)
{
  keisan::Point2 current_position = keisan::Point2(0.0, 0.0);
  keisan::Angle<double> current_orientation = 0.0_deg;

  if (foot_step_planner.foot_steps.size() > 2) {
    double y_offset = 0.0;

    if (status != FootStepPlanner::START) {
      y_offset = next_support == FootStepPlanner::LEFT_FOOT ? -step_y_offset : step_y_offset;
    }

    current_position.x = foot_step_planner.foot_steps[2].position.x;
    current_position.y = foot_step_planner.foot_steps[2].position.y + y_offset;
    current_orientation = foot_step_planner.foot_steps[2].rotation;
  }

  foot_step_planner.plan(
    goal_position, goal_orientation, current_position, current_orientation, next_support, status);

  status = FootStepPlanner::WALKING;

  update_time();
}

void WalkingManager::update_time()
{
  double time = foot_step_planner.foot_steps[1].time;
  lipm.update(time, foot_step_planner.foot_steps);

  if (foot_step_planner.foot_steps[1].support_foot == FootStepPlanner::LEFT_FOOT) {
    if (foot_step_planner.foot_steps[2].support_foot == FootStepPlanner::BOTH_FEET) {
      right_foot_target = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[2].position.x, foot_step_planner.foot_steps[2].position.y,
        foot_step_planner.foot_steps[2].rotation.radian());
    } else {
      right_foot_target = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[2].position.x,
        foot_step_planner.foot_steps[2].position.y + step_y_offset,
        foot_step_planner.foot_steps[2].rotation.radian());
    }

    right_offset_delta = (right_foot_target - right_offset) / step_frames;
    next_support = FootStepPlanner::RIGHT_FOOT;
  } else if (foot_step_planner.foot_steps[1].support_foot == FootStepPlanner::RIGHT_FOOT) {
    if (foot_step_planner.foot_steps[2].support_foot == FootStepPlanner::BOTH_FEET) {
      left_foot_target = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[2].position.x, foot_step_planner.foot_steps[2].position.y,
        foot_step_planner.foot_steps[2].rotation.radian());
    } else {
      left_foot_target = keisan::Matrix<1, 3>(
        foot_step_planner.foot_steps[2].position.x,
        foot_step_planner.foot_steps[2].position.y - step_y_offset,
        foot_step_planner.foot_steps[2].rotation.radian());
    }

    left_offset_delta = (left_foot_target - left_offset) / step_frames;
    next_support = FootStepPlanner::LEFT_FOOT;
  }

  robot_orientation = foot_step_planner.foot_steps[1].rotation;
}

void WalkingManager::update_joints()
{
  auto com = lipm.pop_front();

  double step_period = round(
    (foot_step_planner.foot_steps[2].time - foot_step_planner.foot_steps[1].time) / time_step);

  auto rotation =
    foot_step_planner.foot_steps[2].rotation - foot_step_planner.foot_steps[1].rotation;
  rotation /= step_period;
  robot_orientation += rotation;

  double ssp_start = round(dsp_duration / (2 * time_step));
  double ssp_end = round(step_period / 2);
  double ssp_duration = ssp_end - ssp_start;

  if (foot_step_planner.foot_steps[1].support_foot == FootStepPlanner::LEFT_FOOT) {
    // Raise or lower right foot
    double diff = step_period - lipm.get_com_trajectory().size();
    if (ssp_start < diff && diff <= ssp_end) {
      right_up += foot_height / ssp_duration;
    } else if (right_up > 0.0) {
      right_up = std::max(right_up - foot_height / ssp_duration, 0.0);
    }

    // Move foot to target position and orientation
    if (diff > ssp_start) {
      right_offset += right_offset_delta;

      if (diff > (ssp_start + ssp_duration * 2)) {
        right_offset = right_foot_target;
      }
    }
  } else if (foot_step_planner.foot_steps[1].support_foot == FootStepPlanner::RIGHT_FOOT) {
    // Raise or lower left foot
    double diff = step_period - lipm.get_com_trajectory().size();
    if (ssp_start < diff && diff <= ssp_end) {
      left_up += foot_height / ssp_duration;
    } else if (left_up > 0.0) {
      left_up = std::max(left_up - foot_height / ssp_duration, 0.0);
    }

    // Move foot to target position and orientation
    if (diff > ssp_start) {
      left_offset += left_offset_delta;

      if (diff > (ssp_start + ssp_duration * 2)) {
        left_offset = left_foot_target;
      }
    }
  }

  auto left_foot_pose = keisan::Matrix<1, 3>(
    left_offset[0][0] - com.position.x, left_offset[0][1] - com.position.y, left_offset[0][2]);

  auto right_foot_pose = keisan::Matrix<1, 3>(
    right_offset[0][0] - com.position.x, right_offset[0][1] - com.position.y, right_offset[0][2]);

  Kinematics::Foot left_foot;
  left_foot.position.x = left_foot_pose[0][0] + foot_offset.x;
  left_foot.position.y = left_foot_pose[0][1] + foot_offset.y;
  left_foot.position.z = left_up + foot_offset.z;
  left_foot.yaw = robot_orientation - keisan::make_radian(left_foot_pose[0][2]);

  Kinematics::Foot right_foot;
  right_foot.position.x = right_foot_pose[0][0] + foot_offset.x;
  right_foot.position.y = right_foot_pose[0][1] - foot_offset.y;
  right_foot.position.z = right_up + foot_offset.z;
  right_foot.yaw = robot_orientation - keisan::make_radian(right_foot_pose[0][2]);

  try {
    kinematics.solve_inverse_kinematics(left_foot, right_foot);

    auto angles = kinematics.get_angles();

    for (auto & joint : joints) {
      uint8_t id = joint.get_id();

      joint.set_position(angles[id].degree());
    }

    robot_position = com.position;
  } catch (const std::exception & e) {
    std::cerr << "Failed to solve inverse kinematics!" << std::endl;
    std::cerr << e.what() << std::endl;
  }
}

void WalkingManager::process()
{
  if (lipm.get_com_trajectory().empty() || status == FootStepPlanner::STOP) {
    remove_steps();
    update_time();
  }

  update_joints();
}

}  // namespace gankenkun
