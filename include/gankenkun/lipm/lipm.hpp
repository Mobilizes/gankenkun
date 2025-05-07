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

#ifndef GANKENKUN__LIPM__LIPM_HPP_
#define GANKENKUN__LIPM__LIPM_HPP_

#include <deque>

#include "gankenkun/walking/planner/foot_step_planner.hpp"
#include "keisan/matrix.hpp"

namespace gankenkun
{

class LIPM
{
public:
  LIPM();
  ~LIPM() {}

  void initialize();

  void update(
    double time, const std::deque<FootStepPlanner::FootStep> & foot_steps, bool reset = false);

  void set_parameters(
    double z, double dt, double period, keisan::Point2 foot_size, int preview_steps);

  double dt;
  double period;
  double z;

  keisan::Point2 zmp_limit;
  int horizon;

  struct COMTrajectory
  {
    keisan::Point2 position;
    keisan::Point2 projected_position;
    keisan::Matrix<3, 1> x_state;
    keisan::Matrix<3, 1> y_state;
  };

  COMTrajectory pop_front();

  const std::deque<COMTrajectory> & get_com_trajectory() const { return com_trajectory; }

private:
  // Outputs
  keisan::Matrix<3, 1> x_state;
  keisan::Matrix<3, 1> y_state;
  std::deque<COMTrajectory> com_trajectory;
};

}  // namespace gankenkun

#endif  // GANKENKUN__LIPM__LIPM_HPP_
