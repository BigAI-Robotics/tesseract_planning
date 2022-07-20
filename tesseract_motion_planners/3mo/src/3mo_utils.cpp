#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <tesseract_command_language/mixed_waypoint.h>
#include <queue>
#include <fmt/ranges.h>

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
tesseract_kinematics::IKSolutions getIKWithOrder(tesseract_kinematics::KinematicGroup::Ptr manip,
                                                 const MixedWaypoint& waypoint,
                                                 const std::string working_frame,
                                                 const Eigen::VectorXd& prev_joints,
                                                 const Eigen::VectorXd& cost_coefficient_input)
{
  std::stringstream ss;
  ss << cost_coefficient_input.transpose();
  CONSOLE_BRIDGE_logDebug("getting ik with heuristic for mixed waypoint with cost coeff %s", ss.str().c_str());
  auto limits = manip->getLimits();
  auto redundancy_indices = manip->getRedundancyCapableJointIndices();
  Eigen::VectorXd cost_coeff;
  if (cost_coefficient_input.size())
  {
    assert(cost_coefficient_input.size() == prev_joints.size());
    cost_coeff = cost_coefficient_input;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("no cost coeff found, setting cost coeffs with 1");
    cost_coeff.setOnes(prev_joints.size());
  }
  // if (!info.has_mixed_waypoint)
  // {
  //   throw std::runtime_error("Instruction waypoint need to have a mixed waypoint.");
  // }
  // MixedWaypoint wp = info.instruction.getWaypoint().as<MixedWaypoint>();

  // ik_with_cost_queue is reversed when inserting elements(larger cost will be poped first)
  std::priority_queue<IKWithCost, std::vector<IKWithCost>, std::greater<IKWithCost>> ik_with_cost_queue;
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  for (auto link_target : waypoint.link_targets)
  {
    std::cout << "adding kin group input: " << link_target.first << "\nlinear\n" << link_target.second.linear()
              << "\ntranslation: " << link_target.second.translation().transpose() << "\nworking frame: " << working_frame
              << std::endl;
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(link_target.second, working_frame, link_target.first));
  }
  int retry = 0;
  while (ik_with_cost_queue.size() < 200)
  {
    Eigen::VectorXd ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
    // std::cout << "ik seed: " << ik_seed.transpose() << std::endl << "ik input size: " << ik_inputs.size() << std::endl;
    // std::cout << fmt::format("kin group joint names: {}", manip->getJointNames()).c_str() << std::endl;
    // std::cout << fmt::format("kin group joint names: {}", manip->getAllPossibleTipLinkNames()).c_str() << std::endl;
    tesseract_kinematics::IKSolutions result;
    try
    {
      result = manip->calcInvKin(ik_inputs, ik_seed);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << "\nDesired link pose should be tip-link pose!" << '\n';
      throw e;
    }
    
    // std::cout << "result length: " << result.size() << std::endl;
    for (const auto& res : result)
    {
      double cost = getIKCost(waypoint, res, prev_joints, cost_coeff);
      if (cost > 0)
        ik_with_cost_queue.emplace(res, cost);

      auto redundant_solutions =
          tesseract_kinematics::getRedundantSolutions<double>(res, limits.joint_limits, redundancy_indices);
      for (const auto& redundant_sol : redundant_solutions)
      {
        cost = getIKCost(waypoint, redundant_sol, prev_joints, cost_coeff);
        if (cost > 0)
          ik_with_cost_queue.emplace(redundant_sol, cost);
      }
    }

    if (ik_with_cost_queue.empty())
    {
      retry++;
      if (retry > 5000)  // TODO: adjust the retry number
        throw std::runtime_error("cannot find valid ik solution");
    }
  }
  std::stringstream buffer;
  buffer << ik_with_cost_queue.top().ik.transpose();
  CONSOLE_BRIDGE_logDebug("best ik: %s\ncost: %f", buffer.str().c_str(), ik_with_cost_queue.top().cost);
  // reverse the ik with cost queue and return
  tesseract_kinematics::IKSolutions solutions;
  while (ik_with_cost_queue.size() > 0)
  {
    // solutions.insert(solutions.begin(), ik_with_cost_queue.top().ik);
    // std::cout << ik_with_cost_queue.top().cost << ", ";
    solutions.push_back(ik_with_cost_queue.top().ik);
    ik_with_cost_queue.pop();
  }
  return solutions;

  // get more than 1000 ik solutions
}

double getIKCost(const MixedWaypoint& wp,
                 const Eigen::VectorXd& target,
                 const Eigen::VectorXd& base,
                 const Eigen::VectorXd& cost_coefficient)
{
  assert(target.size() == base.size() && target.size() == cost_coefficient.size());
  double cost = 0;
  cost += (target - base).cwiseProduct(cost_coefficient).array().abs().sum() * 2;
  for (auto const& joint_target : wp.joint_targets)
  {
    // std::cout << "joint target: " << joint_target.first << " " << joint_target.second << std::endl;
    auto itr = std::find(wp.joint_names.begin(), wp.joint_names.end(), joint_target.first);
    int index = std::distance(wp.joint_names.begin(), itr);
    // std::cout << "index found: " << index << std::endl;
    const double diff = std::abs(target[index] - joint_target.second);
    if (diff > 0.2)
    {
      // std::cout << "diff: " << target[index] << " " << diff << std::endl;
      return -1.0;
    }
    cost += std::pow(diff, 1) * 7;
  }
  return std::abs(cost);
}

tesseract_kinematics::IKSolutions filterCollisionIK(const tesseract_environment::Environment::ConstPtr env,
                                                    tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                                    tesseract_kinematics::IKSolutions ik_input)
{
  tesseract_kinematics::IKSolutions result;
  // check collision
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  Eigen::VectorXd best_solution = ik_input.at(0);
  for (auto const& ik : ik_input)
  {
    contact_result.clear();
    auto current_state = env->getState(kin_group->getJointNames(), ik);
    contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
    contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
    if (contact_result.size() <= best_collision_count)
    {
      best_collision_count = contact_result.size();
      best_solution = ik;
      if (contact_result.size() == 0)
      {
        result.push_back(ik);
      }
    }
  }

  if (result.empty())
    result.push_back(best_solution);

  return result;
}

}  // namespace tesseract_planning