#ifndef TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <map>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
namespace tesseract_planning
{

class MixedWaypoint
{
public:

  MixedWaypoint() = default;

  void print(const std::string& prefix = "") const;

  /*<< ", joint_targets: " << this->joint_targets.size()
<< ", link_targets: " << this->link_targets.size() << std::endl;
}*/

  MixedWaypoint(std::vector<std::string> joint_names) : joint_names(std::move(joint_names)) {}

  bool operator==(const MixedWaypoint& rhs) const;
  bool operator!=(const MixedWaypoint& rhs) const;

  // all joint names for manipulator
  std::vector<std::string> joint_names;
  // target joint name and value
  std::map<std::string, double> joint_targets;
  // target link name and pose
  std::map<std::string, Eigen::Isometry3d> link_targets;

  void addJointTarget(std::string joint_name, double joint_value);
  void addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf);

  // not in use(formerly for generating term info, now deprecated)
  // JointWaypoint extractJointWaypoint() const;
  // CartesianWaypoint extractCartesianWaypoint() const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/);  // NOLINT
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_waypoint_type(MixedWaypoint)
#else
TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning::MixedWaypoint);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
