#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>
#include <tesseract_command_language/mixed_waypoint.h>

namespace tesseract_planning
{

struct IKWithCost
{
  Eigen::VectorXd ik;
  double cost;
  IKWithCost(Eigen::VectorXd ik, double cost) : ik{ std::move(ik) }, cost{ cost } {}
  friend bool operator<(IKWithCost const& ik1, IKWithCost const& ik2) { return ik1.cost < ik2.cost; }
};

bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                 std::string link_name,
                 Eigen::Isometry3d& tf,
                 tesseract_collision::ContactResultMap& contact_results);

tesseract_kinematics::IKSolutions getIKWithHeuristic(const KinematicGroupInstructionInfo& info,
                                                     const Eigen::VectorXd& seed);
double getIKCost(const tesseract_planning::MixedWaypoint& wp,
                 const Eigen::VectorXd& target,
                 const Eigen::VectorXd& base);
}  // namespace tesseract_planning