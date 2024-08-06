/**
 * @file simple_motion_planner.h
 * @brief mixed mobile manipulator optimization motion planner
 *
 * @author Ziyuan Jiao, Yida Niu
 * @version TODO
 * @bug No known bugs
 */
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::MMMOMotionPlanner)
%shared_ptr(tesseract_planning::MMMOMotionPlannerStatusCategory)
#endif  // SWIG

namespace tesseract_planning
{
class MMMOMotionPlannerStatusCategory;

/**
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 */
class MMMOMotionPlanner : public MotionPlanner
{
public:
  using Ptr = std::shared_ptr<MMMOMotionPlanner>;
  using ConstPtr = std::shared_ptr<const MMMOMotionPlanner>;

  /** @brief Construct a basic planner */
  MMMOMotionPlanner(std::string name = profile_ns::MMMO_DEFAULT_NAMESPACE);
  ~MMMOMotionPlanner() override = default;
  MMMOMotionPlanner(const MMMOMotionPlanner&) = delete;
  MMMOMotionPlanner& operator=(const MMMOMotionPlanner&) = delete;
  MMMOMotionPlanner(MMMOMotionPlanner&&) = delete;
  MMMOMotionPlanner& operator=(MMMOMotionPlanner&&) = delete;

  const std::string& getName() const override;

  PlannerResponse solve(const PlannerRequest& request) const override;

  static bool checkUserInput(const PlannerRequest& request);

  bool terminate() override;

  void clear() override;

  MotionPlanner::Ptr clone() const override;

protected:
  std::string name_;
  static MoveInstructionPoly getStartInstruction(const PlannerRequest& request,
                                             const tesseract_scene_graph::SceneState& current_state,
                                             const tesseract_kinematics::JointGroup& manip);

  CompositeInstruction processCompositeInstruction(const CompositeInstruction& instructions,
                                                   MoveInstructionPoly& prev_instruction,
                                                   MoveInstructionPoly& prev_seed,
                                                   const PlannerRequest& request) const;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_SIMPLE_PLANNER_H
