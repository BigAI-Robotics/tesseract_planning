#include <tesseract_collision/core/discrete_contact_manager.h>

namespace tesseract_planning
{
bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                 std::string link_name,
                 Eigen::Isometry3d& tf,
                 tesseract_collision::ContactResultMap& contact_results);
}