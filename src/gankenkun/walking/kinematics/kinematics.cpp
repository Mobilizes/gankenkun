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

#include "gankenkun/walking/kinematics/kinematics.hpp"

namespace gankenkun
{

Gankenkun::Gankenkun(const std::string & path) { load_configuration(path); }

void Gankenkun::load_configuration(const std::string & path)
{
  // Load configuration from the specified path
}

void Gankenkun::solve_inverse_kinematics(const Foot & left_foot, const Foot & right_foot)
{
  double left_x = left_foot.position.x - x_offset;
  double left_y = left_foot.position.y - y_offset;
  double left_z = ankle_length + calf_length + knee_length + thigh_length - left_foot.position.z;

  double left_x2 = left_x * left_foot_yaw.cos() + left_y * left_foot.yaw.sin();
  double left_y2 = -left_x * left_foot_yaw.sin() + left_y * left_foot.yaw.cos();
  double left_z2 = left_z - ankle_length;

  // Hip roll angle
  double hip_roll = std::atan2(left_z2, left_y2);

  double left2 = left_y2 * left_y2 + left_z2 * left_z2;
  double left_z3 = std::sqrt(std::max(0.0, left2 - left_x2 * left_x2)) - knee_length;

  double pitch = std::atan2(left_x2, left_z3);
  double length = std::hypot(left_x2, left_z3);
  double knee_disp = acos((std::min(std::max(length / (2.0 * thigh_length), -1.0), 1.0)));

  // Hip pitch angle
  double hip_pitch = -pitch - knee_disp;

  // Knee pitch angle
  double knee_pitch = -pitch + knee_disp;

  angles[JointId::LEFT_HIP_YAW] = keisan::make_radian(hip_yaw);
  angles[JointId::LEFT_HIP_ROLL] = keisan::make_radian(hip_roll);
  angles[JointId::LEFT_HIP_PITCH] = keisan::make_radian(-hip_pitch);
  angles[JointId::LEFT_UPPER_KNEE] = keisan::make_radian(hip_pitch);
  angles[JointId::LEFT_LOWER_KNEE] = keisan::make_radian(knee_pitch);
  angles[JointId::LEFT_ANKLE_PITCH] = 0.0;  // TODO: Add offset from param left_foot pitch
  angles[JointId::LEFT_ANKLE_ROLL] =
    keisan::make_radian(-hip_roll);  // TODO: Add offset from param left_foot roll

  double right_x = right_foot.position.x - x_offset;
  double right_y = right_foot.position.y + y_offset;
  double right_z = ankle_length + calf_length + knee_length + thigh_length - right_foot.position.z;

  double right_x2 = right_x * right_foot_yaw.cos() + right_y * right_foot_yaw.sin();
  double right_y2 = -right_x * right_foot_yaw.sin() + right_y * right_foot_yaw.cos();
  double right_z2 = right_z - ankle_length;

  // Hip roll angle
  hip_roll = std::atan2(right_z2, right_y2);

  double right2 = right_y2 * right_y2 + right_z2 * right_z2;
  double right_z3 = std::sqrt(std::max(0.0, right2 - right_x2 * right_x2)) - knee_length;

  pitch = std::atan2(right_x2, right_z3);
  length = std::hypot(right_x2, right_z3);
  knee_disp = acos((std::min(std::max(length / (2.0 * thigh_length), -1.0), 1.0)));

  // Hip pitch angle
  hip_pitch = -pitch - knee_disp;

  // Knee pitch angle
  knee_pitch = -pitch + knee_disp;

  angles[JointId::RIGHT_HIP_YAW] = keisan::make_radian(hip_yaw);
  angles[JointId::RIGHT_HIP_ROLL] = keisan::make_radian(hip_roll);
  angles[JointId::RIGHT_HIP_PITCH] = keisan::make_radian(hip_pitch);
  angles[JointId::RIGHT_UPPER_KNEE] = keisan::make_radian(-hip_pitch);
  angles[JointId::RIGHT_LOWER_KNEE] = keisan::make_radian(knee_pitch);
  angles[JointId::RIGHT_ANKLE_PITCH] = 0.0;  // TODO: Add offset from param right_foot pitch
  angles[JointId::RIGHT_ANKLE_ROLL] =
    keisan::make_radian(-hip_roll);  // TODO: Add offset from param right_foot roll
}

}  // namespace gankenkun
