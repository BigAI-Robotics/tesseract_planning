/**
 * @file null_waypoint.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/mixed_waypoint.h>

namespace tesseract_planning
{
void MixedWaypoint::print(const std::string& prefix) const
{
  {
    std::cout << prefix << "Mixed WP: joint_names: ";
    for (auto name : joint_names)
    {
      std::cout << name << ", ";
    }
  }
}

void MixedWaypoint::addJointTarget(std::string joint_name, double joint_value)
{
  joint_targets[joint_name] = joint_value;
}

void MixedWaypoint::addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf)
{
  link_targets[link_name] = link_tf;
}

void MixedWaypoint::addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf)
{
  link_constraints[link_name] = link_tf;
}

bool MixedWaypoint::operator==(const MixedWaypoint& /*rhs*/) const { return true; }
bool MixedWaypoint::operator!=(const MixedWaypoint& /*rhs*/) const { return false; }

template <class Archive>
void MixedWaypoint::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MixedWaypoint)
TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::MixedWaypoint);
