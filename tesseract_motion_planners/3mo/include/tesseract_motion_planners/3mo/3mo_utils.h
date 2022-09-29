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
  friend bool operator>(IKWithCost const& ik1, IKWithCost const& ik2) { return ik1.cost > ik2.cost; }
};

bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                 std::string link_name,
                 Eigen::Isometry3d& tf,
                 tesseract_collision::ContactResultMap& contact_results);

tesseract_kinematics::IKSolutions getIKs(tesseract_kinematics::KinematicGroup::Ptr manip,
                                         const MixedWaypointPoly& waypoint,
                                         const std::string working_frame,
                                         double tolerance = 0.2);

std::vector<std::pair<Eigen::VectorXd, double>> getIKsWithCost(tesseract_kinematics::KinematicGroup::Ptr manip,
                                                               const MixedWaypointPoly& waypoint,
                                                               const std::string working_frame,
                                                               const Eigen::VectorXd& prev_joints,
                                                               const Eigen::VectorXd& cost_coeff = Eigen::VectorXd());

double getIKCost(const tesseract_planning::MixedWaypointPoly& wp,
                 const Eigen::VectorXd& target,
                 const Eigen::VectorXd& base,
                 const Eigen::VectorXd& cost_coeff);

double getIKGoalCost(const Eigen::VectorXd& ik, const MixedWaypointPoly& wp, double tolerance);

std::size_t getIKCollisionCount(const tesseract_environment::Environment::ConstPtr env,
                                tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                Eigen::VectorXd joints);

tesseract_kinematics::IKSolutions filterCollisionIK(tesseract_environment::Environment::ConstPtr env,
                                                    tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                                    tesseract_kinematics::IKSolutions ik_input);

std::vector<std::pair<Eigen::VectorXd, double>>
filterCollisionIK(tesseract_environment::Environment::ConstPtr env,
                  tesseract_kinematics::KinematicGroup::Ptr kin_group,
                  std::vector<std::pair<Eigen::VectorXd, double>> ik_input);
}  // namespace tesseract_planning