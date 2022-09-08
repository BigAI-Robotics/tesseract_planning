/**
 * @file 3mo_planner_simple_plan_profile.cpp
 * @brief
 *
 * @author Tyler Marr
 * @date Septemeber 16, 2020
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

#include <tesseract_motion_planners/3mo/profile/3mo_planner_ik_plan_profile.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace tesseract_kinematics;

namespace tesseract_planning
{

MMMOPlannerIKPlanProfile::MMMOPlannerIKPlanProfile(int min_steps,
                                                   double state_longest_valid_segment,
                                                   double translation_longest_valid_segment_length,
                                                   double rotation_longest_valid_segment_length)
  : min_steps(min_steps)
  , state_longest_valid_segment_length(state_longest_valid_segment)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
{
  // std::cout << "map step size:" << map_.step_size << std::endl;
}

CompositeInstruction MMMOPlannerIKPlanProfile::generate(const PlanInstruction& prev_instruction,
                                                        const MoveInstruction& /*prev_seed*/,
                                                        const PlanInstruction& base_instruction,
                                                        const Instruction& /*next_instruction*/,
                                                        const PlannerRequest& request,
                                                        const ManipulatorInfo& global_manip_info) const
{
  CONSOLE_BRIDGE_logDebug("generating mmmo ik plan profile...");
  KinematicGroupInstructionInfo info1(prev_instruction, request, global_manip_info);
  KinematicGroupInstructionInfo info2(base_instruction, request, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    if (info2.has_mixed_waypoint)
      return stateJointMixedWaypoint(info1, info2, request);
  return stateJointJointWaypoint(info1, info2, request);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypoint(info1, info2, request);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypoint(info1, info2);

  return stateCartCartWaypoint(info1, info2, request);
}

CompositeInstruction MMMOPlannerIKPlanProfile::stateJointMixedWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                       const KinematicGroupInstructionInfo& base,
                                                                       const PlannerRequest& request) const
{
  CONSOLE_BRIDGE_logInform("generating joint -> mixed waypoint IK composite instruction...");
  const Eigen::VectorXd j1 = prev.extractJointPosition();
  std::stringstream ss;
  ss << j1.transpose();
  CONSOLE_BRIDGE_logDebug("prev state: %s", ss.str().c_str());

  // calculate possible iks with heuristic
  auto cloned_env = request.env->clone();
  tesseract_kinematics::KinematicGroup::Ptr kin_group = std::move(cloned_env->getKinematicGroup(prev.manip->getName()));
  MixedWaypoint wp = base.instruction.getWaypoint().as<MixedWaypoint>();

  if (!wp.link_targets.empty())
  {
    CONSOLE_BRIDGE_logWarn("link target specified for 3mo planner ik plan profile! ignoring.");
  }

  if (wp.joint_targets_.size() != 1)
    throw std::runtime_error("waypoint joint target size != 1");
  std::pair<std::string, double> jt;
  for (auto const& jt_ : wp.joint_targets_)
  {
    jt.first = jt_.first;
    jt.second = jt_.second;
  }
  auto current_joint_value = cloned_env->getState().joints.at(jt.first);
  auto joint_names = kin_group->getJointNames();  // joint names with target joint
  joint_names.push_back(jt.first);
  Eigen::MatrixXd result(joint_names.size(), min_steps + 1);
  result.row(joint_names.size() - 1) = Eigen::VectorXd::LinSpaced(min_steps + 1, current_joint_value, jt.second);
  for (int i = 0; i < min_steps + 1; ++i)
  {
    Eigen::VectorXd ik_seed = cloned_env->getCurrentJointValues(kin_group->getJointNames());
    Eigen::VectorXd joint_value;
    joint_value.resize(1);
    joint_value(0) = result.row(joint_names.size() - 1)(i);
    cloned_env->setState({ jt.first }, joint_value);
    auto ee_target = cloned_env->getLinkTransform(attach_location_link) * local_joint_origin_transform;
    tesseract_kinematics::KinGroupIKInput ik_input(ee_target, "world", "robotiq_arg2f_base_link");
    Eigen::VectorXd ik_result;
    for (int tries = 0; tries < 200; tries++)
    {
      auto res = kin_group->calcInvKin({ ik_input }, ik_seed);
      tesseract_collision::ContactResultMap contact_results;
      if (!res.size())
        continue;
      for (auto const& ik : res)
      {
        cloned_env->setState(kin_group->getJointNames(), ik);
        cloned_env->getDiscreteContactManager()->contactTest(contact_results,
                                                             tesseract_collision::ContactTestType::ALL);
        if (contact_results.size())
        {
          continue;
        }
        ik_result = ik;
        break;
      }
      if (!ik_result.size())
        continue;
      break;
    }
    if (!ik_result.size())
      throw std::runtime_error("cannot find valid ik when generating ik path.");
    for (int j = 0; j < joint_names.size() - 1; ++j)
    {
      result.col(i)(j) = ik_result(j);
    }
  }
  // put ik result into xxx
  std::cout << "ik: " << result << std::endl;

  // get joint joint seed
  // auto states = getJointJointSeed(j1, joint_target, request, kin_group);
  CompositeInstruction interpolated_composite = getInterpolatedComposite(joint_names, result, base.instruction);

  return interpolated_composite;
}

CompositeInstruction MMMOPlannerIKPlanProfile::stateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                       const KinematicGroupInstructionInfo& base,
                                                                       const PlannerRequest& request) const
{
  // trans dist, rot dist, joint dist is not in use for now
  CONSOLE_BRIDGE_logInform("generating joint -> joint waypoint composite instruction...");
  // get kin group
  auto joint_target = base.extractJointPosition();
  auto joint_start = prev.extractJointPosition();
  KinematicGroup::Ptr kin_group = std::move(request.env->getKinematicGroup(prev.manip->getName()));
  auto states = getJointJointSeed(joint_start, joint_target, request, kin_group);
  return getInterpolatedComposite(kin_group->getJointNames(), states, base.instruction);
}

CompositeInstruction MMMOPlannerIKPlanProfile::stateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                      const KinematicGroupInstructionInfo& base,
                                                                      const PlannerRequest& request) const
{
  // Calculate FK for start
  CONSOLE_BRIDGE_logDebug("state joint cart waypoint called");
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  KinematicGroup::Ptr kin_group = std::move(request.env->getKinematicGroup(prev.manip->getName()));
  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j2_final = getClosestJointSolution(base, j1);
  if (j2_final.size() != 0)
  {
    double joint_dist = (j2_final - j1).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Linearly interpolate in joint space
    // Eigen::MatrixXd states = interpolate(j1, j2_final, steps);
    auto states = getJointJointSeed(j1, j2_final, request, kin_group);
    return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction MMMOPlannerIKPlanProfile::stateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                      const KinematicGroupInstructionInfo& base) const
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j1_final = getClosestJointSolution(prev, j2);
  if (j1_final.size() != 0)
  {
    double joint_dist = (j2 - j1_final).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1_final, j2, steps);
    return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction MMMOPlannerIKPlanProfile::stateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                     const KinematicGroupInstructionInfo& base,
                                                                     const PlannerRequest& request) const
{
  // Get IK seed
  Eigen::VectorXd seed = request.env_state.getJointValues(base.manip->getJointNames());
  tesseract_common::enforcePositionLimits(seed, base.manip->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  std::array<Eigen::VectorXd, 2> sol = getClosestJointSolution(prev, base, seed);

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    double joint_dist = (sol[1] - sol[0]).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = interpolate(sol[0], sol[1], steps);
  }
  else if (sol[0].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = sol[0].replicate(1, steps + 1);
  }
  else if (sol[1].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = sol[1].replicate(1, steps + 1);
  }
  else
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    states = seed.replicate(1, steps + 1);
  }

  // Convert to MoveInstructions
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}
}  // namespace tesseract_planning
