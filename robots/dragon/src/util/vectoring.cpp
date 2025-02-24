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

#include <dragon/util/vectoring.h>
#include <dragon/model/full_vectoring_robot_model.h>
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

  void interfere::getBounds(Dragon::FullVectoringRobotModel* robot_model, PrimeBoundMap& prime_bound_map, \
                           std::vector<int>& roll_locked_gimbal, std::vector<double>& gimbal_nominal_angles)
  {
    int rotor_num = robot_model->getRotorNum();
    std::string thrust_link = robot_model->getThrustLinkName();
    const auto& seg_tf_map = robot_model->getSegmentsTf();

    if (seg_tf_map.size() == 0) return;

    double angle_max_thresh = 1.2; // TODO: parameter
    double collision_padding_rate = 1.2; // TODO: parameter
    double roll_angle_thresh = 0.4;  // TODO: parameter

    prime_bound_map = PrimeBoundMap{};
    std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> interfere_raw_map{};
    std::map<int, std::vector<std::pair<double, double>>> bounds_map{};

    auto tangent_calc = [] (auto r1, auto r2, auto rel_pos)
                        {
                          double x1 = 0;
                          double y1 = 0;
                          double x2 = rel_pos.x();
                          double y2 = rel_pos.z();

                          double b = y1 - y2;

                          double a1 = x2 + r2 - x1;
                          double l1 = sqrt(pow(a1, 2) + pow(b, 2));
                          double phi1 = atan2(r1, l1);
                          double theta1 = atan2(a1, b) + phi1;
                          theta1 = -theta1;

                          double a2 = x2 - r2 - x1;
                          double l2 = sqrt(pow(a2, 2) + pow(b, 2));
                          double phi2 = atan2(r1, l2);
                          double theta2 = atan2(a2, b) - phi2;
                          theta2 = -theta2;

                          return std::make_pair(theta1, theta2);
                        };

    std::stringstream ss_map;
    ss_map << "\n";

    for(int i = 0; i < rotor_num; i++)
      {
        std::string rotor_i = thrust_link + std::to_string(i + 1);
        auto pose_i = seg_tf_map.at(rotor_i); // pose w.r.t. Baselink (Root)

        std::vector<std::pair<int, Eigen::Vector3d>> bound_list;

        for (int j = 0; j < rotor_num; j++)
          {
            if (i == j) continue;


            std::string rotor_j = thrust_link + std::to_string(j + 1);
            auto pose_j = seg_tf_map.at(rotor_j); // pose w.r.t. Baselink (Root)

            // relativel position from the i-th rotor
            auto rel_pose = pose_i.Inverse() * pose_j;
            auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

            // skip if the j-th rotor is not align in the same plane;
            double roll_angle = atan2(fabs(rel_pos.y()), fabs(rel_pos.z()));
            if (roll_angle > roll_angle_thresh) continue;

            double pitch_angle = atan2(-rel_pos.x(), -rel_pos.z());
            // skip if the j-th rotor is far from i-th
            if (fabs(pitch_angle) > angle_max_thresh) continue;

            // calculate the bound angle
            double r = robot_model->getEdfRadius() * collision_padding_rate;
            auto res = tangent_calc(r, r, rel_pos);
            double theta1 = res.first;
            double theta2 = res.second;

            Eigen::Vector3d angle_v(pitch_angle, theta1, theta2);
            bound_list.push_back(std::make_pair(j, angle_v));
          }

        // check the overlap with center link if it exists
        std::string name("center_link");
        auto it = seg_tf_map.find(name);
        if (it != seg_tf_map.end())
          {
            auto pose_c = it->second; // pose w.r.t. Baselink (Root)
            auto rel_pose = pose_i.Inverse() * pose_c; // relativel position from the i-th rotor
            auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

            double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle

            if (fabs(angle) < angle_max_thresh)
              {
                // calculate the bound angle
                auto geo = robot_model->getUrdfModel().getLink(name)->collision->geometry;
                auto type = geo->type;

                double r_r = robot_model->getEdfRadius() * collision_padding_rate;
                double r_c = 0;

                if (type == urdf::Geometry::CYLINDER)
                  {
                    r_c = std::dynamic_pointer_cast<urdf::Cylinder>(geo)->radius;
                  }
                else if (type == urdf::Geometry::SPHERE)
                  {
                    r_c = std::dynamic_pointer_cast<urdf::Sphere>(geo)->radius;
                  }
                else if (type == urdf::Geometry::BOX)
                  {
                    auto dim = std::dynamic_pointer_cast<urdf::Box>(geo)->dim;
                    r_c = std::hypot(dim.x, dim.y);
                  }
                else
                  {
                    ROS_WARN_THROTTLE(1.0, "Currently, urdf geometry type %d is not supported for rotor interference avoidance", type);
                  }

                if (r_c > 0)
                  {
                    auto res = tangent_calc(r_r, r_c, rel_pos);
                    double theta1 = res.first;
                    double theta2 = res.second;
                    Eigen::Vector3d angle_v(angle, theta1, theta2);
                    bound_list.push_back(std::make_pair(-1, angle_v));
                  }
              }
          }

        // check the overlap with extra module if it exists
        int extra_cnt = 0;
        for (const auto& it: robot_model->getExtraModuleMap())
          {
            auto pose_parent = seg_tf_map.find(it.second.first.getName())->second;
            auto pose_e = pose_parent * it.second.first.getFrameToTip(); // pose w.r.t. Baselink (Root)
            auto rel_pose = pose_i.Inverse() * pose_e; // relativel position from the i-th rotor
            auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

            double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle
            if (fabs(angle) > angle_max_thresh) continue;

            // calculate the bound angle
            double r_e = 0;
            auto dim = it.second.second;
            if (dim(1) == 0 && dim(2) == 0) r_e = dim(0); // sphere
            if (dim(1) > 0 && dim(2) == 0) r_e = dim(1); // clyinder
            if (dim(1) > 0 && dim(2) > 0) r_e = std::hypot(dim(0), dim(1)); // box

            double r_r = robot_model->getEdfRadius() * collision_padding_rate;

            auto res = tangent_calc(r_r, r_e, rel_pos);
            double theta1 = res.first;
            double theta2 = res.second;

            Eigen::Vector3d angle_v(angle, theta1, theta2);
            // ROS_INFO_STREAM("rotor " << i+1 << ": " << it.first << ": rel_pose" << rel_pos.transpose() << ": angle_v: " << angle_v.transpose());
            extra_cnt ++;
            bound_list.push_back(std::make_pair(-1 - extra_cnt, angle_v));
          }

        if (bound_list.size() == 0) continue;
        interfere_raw_map.insert(std::make_pair(i, bound_list));

        // update bounds
        std::vector<std::pair<double, double>> bounds {std::make_pair(-M_PI/2, M_PI/2)};
        double area_thresh = 0.4; // TODO: rosparam
        for (auto& new_bound : bound_list)
          {
            double theta1, theta2;
            if (new_bound.second(1) < new_bound.second(2))
              {
                theta1 = new_bound.second(1);
                theta2 = new_bound.second(2);
              }
            else
              {
                theta1 = new_bound.second(2);
                theta2 = new_bound.second(1);
              }

            std::vector<std::pair<double, double>> bounds_temp {};
            for (auto& curr_bound: bounds)
              {
                double curr_theta1 = curr_bound.first;
                double curr_theta2 = curr_bound.second;

                // 6 cases:
                if (theta2 < curr_theta1)
                  {
                    bounds_temp.push_back(curr_bound);
                  }

                if (theta1 < curr_theta1 && theta2 > curr_theta1 && theta2 < curr_theta2)
                  {
                    if (curr_theta2 - theta2 < area_thresh) continue;

                    bounds_temp.push_back(std::make_pair(theta2, curr_theta2));
                  }

                if (theta1 <= curr_theta1 && theta2 >= curr_theta2)
                  {
                    // skip
                    continue;
                  }

                if (theta1 > curr_theta1 && theta2 < curr_theta2)
                  {
                    if (theta1 - curr_theta1 > area_thresh)
                      {
                        bounds_temp.push_back(std::make_pair(curr_theta1, theta1));
                      }
                    if (curr_theta2 - theta2 > area_thresh)
                      {
                        bounds_temp.push_back(std::make_pair(theta2, curr_theta2));
                      }
                  }

                if (theta1 > curr_theta1 && theta1 < curr_theta2 && theta2 > curr_theta2)
                  {
                    if (theta1 - curr_theta1 < area_thresh) continue;

                    bounds_temp.push_back(std::make_pair(curr_theta1, theta1));
                  }

                if (theta1 > curr_theta2)
                  {
                    bounds_temp.push_back(curr_bound);
                  }
              }

            if (bounds_temp.size() == 0) continue;

            bounds = bounds_temp;
          }
        bounds_map.insert(std::make_pair(i, bounds));

        double lower = 1e6;
        double upper = 0;
        for (const auto& bound: bounds)
          {
            double theta1 = bound.first;
            double theta2 = bound.second;

            if (fabs(theta1) < fabs(lower))
              {
                lower = theta1;
                upper = theta2;
              }

            if (fabs(theta2) < fabs(lower))
              {
                lower = theta2;
                upper = theta1;
              }
          }

        // skip if the range is enough for normal tilting
        if (lower * upper < 0 && fabs(lower) > area_thresh / 2) continue;

        // add the offset of the gimbal nominal angle
        lower += gimbal_nominal_angles.at(2 * i + 1);
        upper += gimbal_nominal_angles.at(2 * i + 1);
        prime_bound_map.insert(std::make_pair(i, std::make_pair(lower, upper)));


        if (roll_locked_gimbal.at(i) == 0)
          {
            roll_locked_gimbal.at(i) = 1;
            gimbal_nominal_angles.at(2 * i) = 0;
          }
      }

    if (interfere_raw_map.size() > 0)
      {
        for (const auto& it: interfere_raw_map)
          {
            int rotor_id = it.first;
            ss_map << "rotor" << rotor_id+1 << ": \n";
            for(const auto& bound: it.second)
              {
                int index = bound.first;
                if (index >= 0)
                  {
                    ss_map << "\t rotor" << bound.first + 1 << ": " << bound.second.transpose() << " \n";
                  }
                else if (index == -1)
                  {
                    ss_map << "\t center link: " << bound.second.transpose() << " \n";
                  }
                else
                  {
                    ss_map << "\t extra module: " << bound.second.transpose() << " \n";
                  }
              }

            for(const auto& bound: bounds_map.at(rotor_id))
              {
                ss_map << "\t candidate bound: (" << bound.first << ", " << bound.second << ") \n";
              }

            auto bound = prime_bound_map.find(rotor_id);
            if (bound != prime_bound_map.end())
              {
                ss_map << "\t prime bound: (" << bound->second.first << ", " << bound->second.second << ") \n";
              }
          }

        ROS_INFO_STREAM_THROTTLE(1.0, "\033[32m" << ss_map.str() << "\033[0m");
      }
  }

};
