#ifndef TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>

namespace tesseract_planning
{
class MixedWaypoint
{
public:
  void print(const std::string& prefix = "") const;

  bool operator==(const MixedWaypoint& rhs) const;
  bool operator!=(const MixedWaypoint& rhs) const;

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
