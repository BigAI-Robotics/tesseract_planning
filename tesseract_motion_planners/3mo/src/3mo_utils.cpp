#include <tesseract_motion_planners/3mo/3mo_utils.h>

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
}  // namespace tesseract_planning