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

#ifndef GANKENKUN__WALKING__PLANNER__FOOT_STEP_PLANNER_HPP_
#define GANKENKUN__WALKING__PLANNER__FOOT_STEP_PLANNER_HPP_

#include <deque>

#include "keisan/angle.hpp"
#include "keisan/geometry/point_2.hpp"

namespace gankenkun
{

class FootStepPlanner
{
public:
  enum { LEFT_FOOT = 0, RIGHT_FOOT = 1, BOTH_FEET = 2 };

  enum { START = 0, WALKING = 1, STOP = 2 };

  struct FootStep
  {
    double time;
    keisan::Point2 position;
    keisan::Angle<double> rotation;
    int support_foot;
  };

  FootStepPlanner();

  void set_parameters(
    const keisan::Point2 & max_stride, const keisan::Angle<double> & max_rotation, double period,
    double width);

  void plan(
    const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
    keisan::Point2 & current_position, keisan::Angle<double> & current_orientation,
    int next_support, int status);

  void print_foot_steps();

  std::deque<FootStep> foot_steps;

private:
  keisan::Point2 max_stride;
  keisan::Angle<double> max_rotation;
  double period;
  double width;
};

}  // namespace gankenkun

#endif  // GANKENKUN__WALKING__PLANNER__FOOT_STEP_PLANNER_HPP_
