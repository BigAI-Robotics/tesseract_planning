#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <tesseract_command_language/mixed_waypoint.h>
#include <queue>

namespace tesseract_planning
{
bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                 std::string link_name,
                 Eigen::Isometry3d& tf,
                 tesseract_collision::ContactResultMap& contact_results)
{
  discrete_contact_manager->setCollisionObjectsTransform(link_name, tf);
  discrete_contact_manager->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);
  for (auto& collision : contact_results)
  {
    // std::cout << collision.first.first << ":\t" << collision.first.second <<
    // std::endl;
    if (collision.first.first == "base_link" || collision.first.second == "base_link")
    {
      return false;
    }
  }
  return true;
}

// seed here is the initial joint pose, not actual seed used for kinematics.
tesseract_kinematics::IKSolutions getIKWithHeuristic(const KinematicGroupInstructionInfo& info,
                                                     const Eigen::VectorXd& seed)
{
  CONSOLE_BRIDGE_logDebug("getting ik with heuristic for mixed waypoint");
  auto limits = info.manip->getLimits();
  auto redundancy_indices = info.manip->getRedundancyCapableJointIndices();
  if (!info.has_mixed_waypoint)
  {
    throw std::runtime_error("Instruction waypoint need to have a mixed waypoint.");
  }
  MixedWaypoint wp = info.instruction.getWaypoint().as<MixedWaypoint>();

  // ik_with_cost_queue is reversed when inserting elements(larger cost will be poped first)
  std::priority_queue<IKWithCost> ik_with_cost_queue;
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  for (auto link_target : wp.link_targets)
  {
    ik_inputs.push_back(
        tesseract_kinematics::KinGroupIKInput(link_target.second, info.working_frame, link_target.first));
  }
  int retry = 0;
  while (ik_with_cost_queue.size() < 1000)
  {
    Eigen::VectorXd ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
    tesseract_kinematics::IKSolutions result = info.manip->calcInvKin(ik_inputs, ik_seed);
    for (const auto& res : result)
    {
      ik_with_cost_queue.emplace(res, getIKCost(wp, res, seed));
      auto redundant_solutions =
          tesseract_kinematics::getRedundantSolutions<double>(res, limits.joint_limits, redundancy_indices);
      for (const auto& redundant_sol : redundant_solutions)
      {
        ik_with_cost_queue.emplace(redundant_sol, getIKCost(wp, redundant_sol, seed));
      }
    }
    // for (const auto& res : result)
    // {
    //   sol.push_back(res);
    //   auto redundant_solutions =
    //       tesseract_kinematics::getRedundantSolutions<double>(res, limits.joint_limits, redundancy_indices);
    //   sol.insert(sol.end(), redundant_solutions.begin(), redundant_solutions.end());
    // }
    if (ik_with_cost_queue.empty())
    {
      retry++;
      if (retry > 500)  // TODO: adjust the retry number
        throw std::runtime_error("cannot find valid ik solution");
    }
  }
  // reverse the ik with cost queue and return
  tesseract_kinematics::IKSolutions solutions;
  while (ik_with_cost_queue.size() > 0)
  {
    solutions.insert(solutions.begin(), ik_with_cost_queue.top().ik);
    // solutions.push_back(ik_with_cost_queue.top().ik);
    ik_with_cost_queue.pop();
  }
  return solutions;

  // get more than 1000 ik solutions
}

double getIKCost(const MixedWaypoint& wp, const Eigen::VectorXd& target, const Eigen::VectorXd& base)
{
  double cost = 0;
  cost += (target - base).array().abs().sum();
  for (auto const& joint_target : wp.joint_targets)
  {
    auto itr = std::find(wp.joint_names.begin(), wp.joint_names.end(), joint_target.first);
    int index = std::distance(wp.joint_names.begin(), itr);
    cost += std::pow(std::abs(target[index] - joint_target.second), 1) * 20;
  }
  return cost;
}

}  // namespace tesseract_planning