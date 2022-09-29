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

tesseract_kinematics::IKSolutions getIKs(tesseract_kinematics::KinematicGroup::Ptr manip,
                                         const MixedWaypointPoly& waypoint,
                                         const std::string working_frame,
                                         double tolerance)
{
  CONSOLE_BRIDGE_logDebug("getting iks...");
  auto limits = manip->getLimits();
  auto redundancy_indices = manip->getRedundancyCapableJointIndices();
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  for (auto link_target : waypoint.getLinkTargets())
  {
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(link_target.second, working_frame, link_target.first));
  }
  tesseract_kinematics::IKSolutions solutions;
  int retry = 0;
  while (solutions.size() < 400)
  {
    Eigen::VectorXd ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
    // std::cout << "ik seed: " << ik_seed.transpose() << std::endl << "ik input size: " << ik_inputs.size() <<
    // std::endl; std::cout << fmt::format("kin group joint names: {}", manip->getJointNames()).c_str() << std::endl;
    // std::cout << fmt::format("kin group joint names: {}", manip->getAllPossibleTipLinkNames()).c_str() << std::endl;
    tesseract_kinematics::IKSolutions result;
    try
    {
      result = manip->calcInvKin(ik_inputs, ik_seed);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      CONSOLE_BRIDGE_logError("Can't find desired link name in kinematic chain.");
      throw e;
    }

    // std::cout << "result length: " << result.size() << std::endl;
    for (const auto& res : result)
    {
      if (getIKGoalCost(res, waypoint, tolerance) >= 0)
        solutions.push_back(res);
      auto redundant_solutions =
          tesseract_kinematics::getRedundantSolutions<double>(res, limits.joint_limits, redundancy_indices);
      for (const auto& redundant_sol : redundant_solutions)
      {
        if (getIKGoalCost(redundant_sol, waypoint, tolerance) >= 0)
          solutions.push_back(redundant_sol);
      }
    }

    if (solutions.empty())
    {
      retry++;
      if (retry > 5000)  // TODO: adjust the retry number
        throw std::runtime_error("cannot find valid ik solution");
    }
  }

  return solutions;
}

// seed here is the initial joint pose, not actual seed used for kinematics.
std::vector<std::pair<Eigen::VectorXd, double>> getIKsWithCost(tesseract_kinematics::KinematicGroup::Ptr manip,
                                                               const MixedWaypointPoly& waypoint,
                                                               const std::string working_frame,
                                                               const Eigen::VectorXd& prev_joints,
                                                               const Eigen::VectorXd& cost_coefficient_input)
{
  std::stringstream ss;

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
    CONSOLE_BRIDGE_logInform("no cost coeff found, setting cost coeffs with 1");
    cost_coeff.setOnes(prev_joints.size());
  }
  ss << cost_coeff.transpose();
  CONSOLE_BRIDGE_logInform("getting ik with heuristic for mixed waypoint with cost coeff %s", ss.str().c_str());
  // if (!info.has_mixed_waypoint)
  // {
  //   throw std::runtime_error("Instruction waypoint need to have a mixed waypoint.");
  // }
  // MixedWaypoint wp = info.instruction.getWaypoint().as<MixedWaypoint>();

  // ik_with_cost_queue is reversed when inserting elements(larger cost will be poped first)
  std::priority_queue<IKWithCost, std::vector<IKWithCost>, std::greater<IKWithCost>> ik_with_cost_queue;
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  for (auto link_target : waypoint.getLinkTargets())
  {
    std::stringstream ss;
    ss << link_target.first << ", working frame: " << working_frame
       << ", translation: " << link_target.second.translation().transpose() << ", linear\n"
       << link_target.second.linear();
    CONSOLE_BRIDGE_logDebug("adding kin group input: %s", ss.str().c_str());
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(link_target.second, working_frame, link_target.first));
  }
  int retry = 0;
  while (ik_with_cost_queue.size() < 200)
  {
    Eigen::VectorXd ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
    // std::cout << "ik seed: " << ik_seed.transpose() << std::endl << "ik input size: " << ik_inputs.size() <<
    // std::endl; std::cout << fmt::format("kin group joint names: {}", manip->getJointNames()).c_str() << std::endl;
    // std::cout << fmt::format("kin group joint names: {}", manip->getAllPossibleTipLinkNames()).c_str() << std::endl;
    tesseract_kinematics::IKSolutions result;
    try
    {
      result = manip->calcInvKin(ik_inputs, ik_seed);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      CONSOLE_BRIDGE_logError("Can't find desired link name in kinematic chain.");
      throw e;
    }

    std::cout << "result length: " << result.size() << std::endl;
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
  std::stringstream coeff_buffer;
  coeff_buffer << cost_coeff.transpose();
  CONSOLE_BRIDGE_logDebug("best ik: %s\ncost: %f, coeff: %s",
                          buffer.str().c_str(),
                          ik_with_cost_queue.top().cost,
                          coeff_buffer.str().c_str());
  // reverse the ik with cost queue and return
  std::vector<std::pair<Eigen::VectorXd, double>> solutions;
  while (ik_with_cost_queue.size() > 0)
  {
    // solutions.insert(solutions.begin(), ik_with_cost_queue.top().ik);
    // std::cout << ik_with_cost_queue.top().cost << ", ";
    solutions.push_back(std::make_pair(ik_with_cost_queue.top().ik, ik_with_cost_queue.top().cost));
    ik_with_cost_queue.pop();
  }
  return solutions;

  // get more than 1000 ik solutions
}

double getIKCost(const MixedWaypointPoly& wp,
                 const Eigen::VectorXd& target,
                 const Eigen::VectorXd& base,
                 const Eigen::VectorXd& cost_coefficient)
{
  assert(target.size() == base.size() && target.size() == cost_coefficient.size());
  double cost = 0;
  cost += (target - base).cwiseProduct(cost_coefficient).array().abs().sum() * 2;
  double ik_goal_cost = getIKGoalCost(target, wp, 0.2);
  if (ik_goal_cost < 0)
  {
    return -1.0;
  }
  cost += ik_goal_cost * 7;
  return std::abs(cost);
}

double getIKGoalCost(const Eigen::VectorXd& ik, const MixedWaypointPoly& wp, double tolerance)
{
  double cost = 0;
  for (auto const& jt : wp.getJointIndexTargets())
  {
    const double diff = std::abs(ik[jt.first] - jt.second);
    if (diff > tolerance)
    {
      return -1.0;
    }
    cost += std::pow(diff, 1);
  }
}

std::size_t getIKCollisionCount(const tesseract_environment::Environment::ConstPtr env,
                                tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                Eigen::VectorXd joints)
{
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  auto current_state = env->getState(kin_group->getJointNames(), joints);
  contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
  contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
  // for (auto& collision : contact_result)
  // {
  //   std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
  // }
  return contact_result.size();
}

tesseract_kinematics::IKSolutions filterCollisionIK(const tesseract_environment::Environment::ConstPtr env,
                                                    tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                                    tesseract_kinematics::IKSolutions ik_input)
{
  CONSOLE_BRIDGE_logDebug("filtering ik with collision...");
  tesseract_kinematics::IKSolutions result;
  // check collision
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::ContactResultMap best_contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  Eigen::VectorXd best_solution = ik_input.at(0);
  for (auto const& ik : ik_input)
  {
    contact_result.clear();
    auto current_state = env->getState(kin_group->getJointNames(), ik);
    contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
    contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
    // for (auto& collision : contact_result)
    // {
    //   std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    // }
    if (contact_result.size() <= best_collision_count)
    {
      best_collision_count = contact_result.size();
      best_solution = ik;
      best_contact_result = contact_result;
      if (contact_result.size() == 0)
      {
        result.push_back(ik);
      }
    }
  }

  if (result.empty())
  {
    CONSOLE_BRIDGE_logWarn("ik solution is empty, saving best solution into result. best collision count: %d",
                           best_collision_count);
    for (auto& collision : best_contact_result)
    {
      std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    }
    result.push_back(best_solution);
  }
  CONSOLE_BRIDGE_logDebug("collision free ik: %ld/%ld", ik_input.size(), result.size());

  return result;
}

std::vector<std::pair<Eigen::VectorXd, double>>
filterCollisionIK(tesseract_environment::Environment::ConstPtr env,
                  tesseract_kinematics::KinematicGroup::Ptr kin_group,
                  std::vector<std::pair<Eigen::VectorXd, double>> ik_input)
{
  CONSOLE_BRIDGE_logDebug("filtering ik with collision...");
  std::vector<std::pair<Eigen::VectorXd, double>> result;
  // check collision
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::ContactResultMap best_contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  std::pair<Eigen::VectorXd, double> best_solution = ik_input.at(0);
  for (auto const& ik : ik_input)
  {
    contact_result.clear();
    auto current_state = env->getState(kin_group->getJointNames(), ik.first);
    contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
    contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
    // for (auto& collision : contact_result)
    // {
    //   std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    // }
    if (contact_result.size() <= best_collision_count)
    {
      best_collision_count = contact_result.size();
      best_solution = ik;
      best_contact_result = contact_result;
      if (contact_result.size() == 0)
      {
        result.push_back(ik);
      }
    }
  }

  if (result.empty())
  {
    CONSOLE_BRIDGE_logWarn("ik solution is empty, saving best solution into result. best collision count: %d",
                           best_collision_count);
    for (auto& collision : best_contact_result)
    {
      std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    }
    result.push_back(best_solution);
  }
  CONSOLE_BRIDGE_logDebug("collision free ik: %ld/%ld", ik_input.size(), result.size());

  return result;
}

}  // namespace tesseract_planning