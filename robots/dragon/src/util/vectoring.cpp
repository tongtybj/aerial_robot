// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cmath>

namespace vectoring
{
  void getShortestPath(double& roll_angle, const double prev_roll_angle, \
                       double& pitch_angle, const double prev_pitch_angle)
  {
    double roll_diff = roll_angle - prev_roll_angle;
    double pitch_diff = pitch_angle - prev_pitch_angle;

    // 1. solve the problem of discontinuity of radian
    roll_angle -= std::round(roll_diff / (2 * M_PI)) * 2 * M_PI;
    pitch_angle -= std::round(pitch_diff / (2 * M_PI)) * 2 * M_PI;

    // 2. solve the dual solution issue of 2-DoF gimbal
    // there are two option to achieve the same thrust direction (roll, pitch, roll) ad (roll + PI, pitch + PI)
    roll_diff = roll_angle - prev_roll_angle;
    pitch_diff = pitch_angle - prev_pitch_angle;
    if (fabs(roll_diff) > M_PI/2)
      {
        if (roll_diff > M_PI/2) roll_angle -= M_PI;
        if (roll_diff < -M_PI/2) roll_angle += M_PI;

        if (pitch_angle > 0) pitch_angle = M_PI - pitch_angle;
        else pitch_angle = -M_PI - pitch_angle;
      }

    // 3. solve the problem of discontinuity of radian again
    roll_diff = roll_angle - prev_roll_angle;
    pitch_diff = pitch_angle - prev_pitch_angle;
    if (roll_diff > M_PI) roll_angle -= 2 * M_PI;
    if (roll_diff < - M_PI) roll_angle += 2 * M_PI;
    if (pitch_diff > M_PI) pitch_angle -= 2 * M_PI;
    if (pitch_diff < - M_PI) pitch_angle += 2 * M_PI;
  }
};
