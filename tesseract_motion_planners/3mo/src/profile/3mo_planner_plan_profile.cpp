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

#include <AStar.hpp>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_plan_profile.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace tesseract_kinematics;

namespace tesseract_planning
{
// MMMOPlannerPlanProfile::MMMOPlannerPlanProfile(MapInfo& map,
//                                                int min_steps,
//                                                double state_longest_valid_segment_length,
//                                                double translation_longest_valid_segment_length,
//                                                double rotation_longest_valid_segment_length)
//   : map_(MapInfo(map.map_x, map.map_y, map.step_size))
//   , state_longest_valid_segment_length(state_longest_valid_segment_length)
//   , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
//   , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
//   , min_steps(min_steps)
// {
// }

MMMOPlannerPlanProfile::MMMOPlannerPlanProfile(int x,
                                               int y,
                                               double res,
                                               int min_steps,
                                               double state_longest_valid_segment,
                                               double translation_longest_valid_segment_length,
                                               double rotation_longest_valid_segment_length)
  : map_(MapInfo(x, y, res))
  , min_steps(min_steps)
  , state_longest_valid_segment_length(state_longest_valid_segment)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
{
  // std::cout << "map step size:" << map_.step_size << std::endl;
}

CompositeInstruction MMMOPlannerPlanProfile::generate(const PlanInstruction& prev_instruction,
                                                      const MoveInstruction& /*prev_seed*/,
                                                      const PlanInstruction& base_instruction,
                                                      const Instruction& /*next_instruction*/,
                                                      const PlannerRequest& request,
                                                      const ManipulatorInfo& global_manip_info) const
{
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

CompositeInstruction MMMOPlannerPlanProfile::stateJointMixedWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                     const KinematicGroupInstructionInfo& base,
                                                                     const PlannerRequest& request) const
{
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  // calculate possible iks with heuristic
  auto ik_result = getIKWithHeuristic(base, j1);
  // get joint joint seed
  KinematicGroup::Ptr kin_group = std::move(request.env->getKinematicGroup(prev.manip->getName()));
  auto states = getJointJointSeed(j1, ik_result.at(0), request, kin_group);
  // std::cout << "target joints: " << std::endl
  //           << ik_result.at(0) << std::endl
  //           << "start joints: " << std::endl
  //           << j1 << std::endl;
  return getInterpolatedComposite(kin_group->getJointNames(), states, base.instruction);
}

CompositeInstruction MMMOPlannerPlanProfile::stateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                     const KinematicGroupInstructionInfo& base,
                                                                     const PlannerRequest& request) const
{
  // trans dist, rot dist, joint dist is not in use for now
  // get kin group
  auto joint_target = base.extractJointPosition();
  auto joint_start = prev.extractJointPosition();
  KinematicGroup::Ptr kin_group = std::move(request.env->getKinematicGroup(prev.manip->getName()));
  auto states = getJointJointSeed(joint_start, joint_target, request, kin_group);
  return getInterpolatedComposite(kin_group->getJointNames(), states, base.instruction);
}

// get seed between 2 joint waypoints
Eigen::MatrixXd MMMOPlannerPlanProfile::getJointJointSeed(const Eigen::VectorXd& joint_start,
                                                          const Eigen::VectorXd& joint_target,
                                                          const PlannerRequest& request,
                                                          KinematicGroup::Ptr kin_group) const
{
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_ =
      std::move(request.env->getDiscreteContactManager()->clone());
  for (auto& active_link : discrete_contact_manager_->getActiveCollisionObjects())
  {
    discrete_contact_manager_->enableCollisionObject(active_link);
  }

  std::vector<Eigen::Isometry3d> base_poses;
  int steps = min_steps;
  initBaseTrajectory_(kin_group, discrete_contact_manager_, joint_start, joint_target, base_poses, steps);
  Eigen::MatrixXd states = interpolate(joint_start, joint_target, steps);
  // set base pose from astar
  for (int i = 1; i < steps; ++i)
  {
    states(0, i) = base_poses[i].translation()[0];
    states(1, i) = base_poses[i].translation()[1];
  }
  return states;
}

void MMMOPlannerPlanProfile::initBaseTrajectory_(
    KinematicGroup::Ptr kin_group,
    tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
    const Eigen::VectorXd& joint_start,
    const Eigen::VectorXd& joint_target,
    std::vector<Eigen::Isometry3d>& base_poses,
    int n_steps) const
{
  // std::cout << "map step size: " << map_.step_size << std::endl;
  tesseract_collision::ContactResultMap contact_results;
  // init base trajectory
  Eigen::Isometry3d base_start_pose;
  base_start_pose.setIdentity();
  base_start_pose.translation() = Eigen::Vector3d(joint_start[0], joint_start[1], 0.13);
  Eigen::Isometry3d base_target_pose;
  base_target_pose.setIdentity();
  base_target_pose.translation() = Eigen::Vector3d(joint_target[0], joint_target[1], 0.13);

  int base_x = int(round((base_start_pose.translation()[0] + map_.map_x / 2.0) / map_.step_size));
  int base_y = int(round((base_start_pose.translation()[1] + map_.map_y / 2.0) / map_.step_size));

  int end_x = int(round((base_target_pose.translation()[0] + map_.map_x / 2.0) / map_.step_size));
  int end_y = int(round((base_target_pose.translation()[1] + map_.map_y / 2.0) / map_.step_size));
  Eigen::Isometry3d base_tf;
  std::vector<std::string> link_names = kin_group->getLinkNames();

  for (auto& link_name : link_names)
  {
    if (link_name != "base_link" && link_name != "world")
    {
      if (!discrete_contact_manager->removeCollisionObject(link_name))
      {
        // ROS_WARN("Unable to remove collision object: %s", link_name.c_str());
      }
    }
  }

  AStar::Generator astar_generator;
  astar_generator.setWorldSize({ map_.grid_size_x, map_.grid_size_y });
  astar_generator.setHeuristic(AStar::Heuristic::euclidean);
  astar_generator.setDiagonalMovement(true);

  CONSOLE_BRIDGE_logDebug("map info: %d, %d, stepsize: %f", map_.grid_size_x, map_.grid_size_y, map_.step_size);
  CONSOLE_BRIDGE_logDebug("S/G info: %d, %d / %d, %d", base_x, base_y, end_x, end_y);

  // add collisions
  for (int x = 0; x < map_.grid_size_x; ++x)
  {
    for (int y = 0; y < map_.grid_size_y; ++y)
    {
      base_tf.setIdentity();
      contact_results.clear();
      base_tf.translation() =
          Eigen::Vector3d(-map_.map_x / 2.0 + x * map_.step_size, -map_.map_y / 2.0 + y * map_.step_size, 0.13);
      if (!isEmptyCell(discrete_contact_manager, "base_link", base_tf, contact_results) &&
          (!(x == base_x && y == base_y) && !(x == end_x && y == end_y)))
      {
        // std::cout << "o";
        // std::cout << x << ":\t" << y << std::endl;
        astar_generator.addCollision({ x, y });
      }
      else if (x == base_x && y == base_y)
      {
        // std::cout << "S";
      }
      else if (x == end_x && y == end_y)
      {
        // std::cout << "G";
      }
      else
      {
        // std::cout << "+";
      }
    }
    // std::cout << "" << std::endl;
  }

  // generate path
  std::vector<Eigen::Isometry3d> base_poses_raw;

  auto path = astar_generator.findPath({ base_x, base_y }, { end_x, end_y });
  base_poses_raw.clear();
  base_poses.clear();
  for (auto& coordinate : path)
  {
    Eigen::Isometry3d base_target;
    base_target.setIdentity();
    base_target.translation() = Eigen::Vector3d(
        -map_.map_x / 2.0 + coordinate.x * map_.step_size, -map_.map_y / 2.0 + coordinate.y * map_.step_size, 0.13);
    base_poses_raw.push_back(base_target);
  }
  std::reverse(base_poses_raw.begin(), base_poses_raw.end());

  // CONSOLE_BRIDGE_logDebug("base poses generated: \n");
  // for (auto& pose : base_poses_raw)
  // {
  //   std::cout << pose.translation() << std::endl;
  // }

  // interpolate
  std::vector<double> nsteps_remap;
  for (int i = 0; i < n_steps; ++i)
  {
    nsteps_remap.push_back(i / 1.0 / (n_steps - 1) * ((double)base_poses.size() - 1));
  }

  for (size_t i = 0; i < n_steps; ++i)
  {
    Eigen::Isometry3d pose;
    pose.setIdentity();
    double idx = nsteps_remap[i];
    double idx_1d = 0;
    double rem = modf(idx, &idx_1d);
    double idx_2d = idx_1d + 1;
    size_t idx_1 = size_t(idx_1d);
    size_t idx_2 = size_t(idx_2d);

    double waypoints_len =
        pow(pow(base_poses_raw[idx_2].translation()[0] - base_poses_raw[idx_1].translation()[0], 2.0) +
                pow(base_poses_raw[idx_2].translation()[1] - base_poses_raw[idx_1].translation()[1], 2.0),
            0.5);
    double grad_x = (base_poses_raw[idx_2].translation()[0] - base_poses_raw[idx_1].translation()[0]) / waypoints_len;
    double grad_y = (base_poses_raw[idx_2].translation()[1] - base_poses_raw[idx_1].translation()[1]) / waypoints_len;

    pose.translation()[0] = base_poses_raw[idx_1].translation()[0] + grad_x * rem * waypoints_len;
    pose.translation()[1] = base_poses_raw[idx_1].translation()[1] + grad_y * rem * waypoints_len;
    if (std::isnan(pose.translation()[0]) || std::isnan(pose.translation()[1]))
    {
      pose.translation()[0] = base_poses.back().translation()[0];
      pose.translation()[1] = base_poses.back().translation()[1];
      CONSOLE_BRIDGE_logWarn("nan pose found at step %d", i);
      std::cout << pose.translation() << std::endl;
    }
    base_poses.push_back(pose);
  }
  return;
}

CompositeInstruction MMMOPlannerPlanProfile::stateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
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

CompositeInstruction MMMOPlannerPlanProfile::stateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
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

CompositeInstruction MMMOPlannerPlanProfile::stateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
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
