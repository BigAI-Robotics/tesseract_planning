/**
 * @file trajopt_default_plan_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>

namespace tesseract_planning
{
TrajOptDefaultPlanProfile::TrajOptDefaultPlanProfile(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* cartesian_coeff_element = xml_element.FirstChildElement("CartesianCoefficients");
  const tinyxml2::XMLElement* joint_coeff_element = xml_element.FirstChildElement("JointCoefficients");
  const tinyxml2::XMLElement* term_type_element = xml_element.FirstChildElement("Term");
  const tinyxml2::XMLElement* cnt_error_fn_element = xml_element.FirstChildElement("ConstraintErrorFunctions");

  tinyxml2::XMLError status{ tinyxml2::XMLError::XML_SUCCESS };

  if (cartesian_coeff_element != nullptr)
  {
    std::vector<std::string> cart_coeff_tokens;
    std::string cart_coeff_string;
    status = tesseract_common::QueryStringText(cartesian_coeff_element, cart_coeff_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajOptPlanProfile: Error parsing CartesianCoeff string");

    boost::split(cart_coeff_tokens, cart_coeff_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(cart_coeff_tokens))
      throw std::runtime_error("TrajOptPlanProfile: CartesianCoeff are not all numeric values.");

    cartesian_coeff.resize(static_cast<long>(cart_coeff_tokens.size()));
    for (std::size_t i = 0; i < cart_coeff_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(cart_coeff_tokens[i], cartesian_coeff[static_cast<long>(i)]);
  }

  if (joint_coeff_element != nullptr)
  {
    std::vector<std::string> joint_coeff_tokens;
    std::string joint_coeff_string;
    status = tesseract_common::QueryStringText(joint_coeff_element, joint_coeff_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajOptPlanProfile: Error parsing JointCoeff string");

    boost::split(joint_coeff_tokens, joint_coeff_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(joint_coeff_tokens))
      throw std::runtime_error("TrajOptPlanProfile: JointCoeff are not all numeric values.");

    joint_coeff.resize(static_cast<long>(joint_coeff_tokens.size()));
    for (std::size_t i = 0; i < joint_coeff_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(joint_coeff_tokens[i], joint_coeff[static_cast<long>(i)]);
  }

  if (term_type_element != nullptr)
  {
    auto type = static_cast<int>(trajopt::TermType::TT_CNT);
    status = term_type_element->QueryIntAttribute("type", &type);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajOptPlanProfile: Error parsing Term type attribute.");

    term_type = static_cast<trajopt::TermType>(type);
  }

  if (cnt_error_fn_element != nullptr)
  {
    std::string error_fn_name;
    status = tesseract_common::QueryStringAttribute(cnt_error_fn_element, "type", error_fn_name);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajOptPlanProfile: Error parsing ConstraintErrorFunctions plugin attribute.");

    // TODO: Implement plugin capabilities
  }
}
void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const CartesianWaypointPoly& cartesian_waypoint,
                                      const MoveInstructionPoly& parent_instruction,
                                      const tesseract_common::ManipulatorInfo& manip_info,
                                      const std::vector<std::string>& active_links,
                                      int index) const
{
  assert(!(manip_info.empty() && parent_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(parent_instruction.getManipulatorInfo());

  if (mi.manipulator.empty())
    throw std::runtime_error("TrajOptPlanProfile, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("TrajOptPlanProfile, tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("TrajOptPlanProfile, working_frame is empty!");

  Eigen::Isometry3d tcp_offset = pci.env->findTCPOffset(mi);

  trajopt::TermInfo::Ptr ti{ nullptr };

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  bool is_active_tcp_frame = (std::find(active_links.begin(), active_links.end(), mi.tcp_frame) != active_links.end());
  bool is_static_working_frame =
      (std::find(active_links.begin(), active_links.end(), mi.working_frame) == active_links.end());

  if (cartesian_waypoint.isToleranced())
    CONSOLE_BRIDGE_logWarn("Tolerance cartesian waypoints are not supported in this version of TrajOpt.");

  if ((is_static_working_frame && is_active_tcp_frame) || (!is_active_tcp_frame && !is_static_working_frame))
  {
    ti = createCartesianWaypointTermInfo(index,
                                         mi.working_frame,
                                         cartesian_waypoint.getTransform(),
                                         mi.tcp_frame,
                                         tcp_offset,
                                         cartesian_coeff,
                                         term_type);
  }
  else if (!is_static_working_frame && is_active_tcp_frame)
  {
    ti = createDynamicCartesianWaypointTermInfo(index,
                                                mi.working_frame,
                                                cartesian_waypoint.getTransform(),
                                                mi.tcp_frame,
                                                tcp_offset,
                                                cartesian_coeff,
                                                term_type);
  }
  else
  {
    throw std::runtime_error("TrajOpt, both tcp_frame and working_frame are both static!");
  }

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);

  // Add constraints from error functions if available.
  addConstraintErrorFunctions(pci, index);
}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const JointWaypointPoly& joint_waypoint,
                                      const MoveInstructionPoly& /*parent_instruction*/,
                                      const tesseract_common::ManipulatorInfo& /*manip_info*/,
                                      const std::vector<std::string>& /*active_links*/,
                                      int index) const
{
  trajopt::TermInfo::Ptr ti;
  if (joint_waypoint.isToleranced())
    ti = createTolerancedJointWaypointTermInfo(joint_waypoint.getPosition(),
                                               joint_waypoint.getLowerTolerance(),
                                               joint_waypoint.getUpperTolerance(),
                                               index,
                                               joint_coeff,
                                               term_type);
  else
    ti = createJointWaypointTermInfo(joint_waypoint.getPosition(), index, joint_coeff, term_type);

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);

  // Add constraints from error functions if available.
  addConstraintErrorFunctions(pci, index);
}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const MixedWaypointPoly& mixed_waypoint,
                                      const MoveInstructionPoly& parent_instruction,
                                      const tesseract_common::ManipulatorInfo& manip_info,
                                      const std::vector<std::string>& /*active_links*/,
                                      int index,
                                      bool is_target) const
{
  std::vector<trajopt::TermInfo::Ptr> term_infos;
  // CONSOLE_BRIDGE_logDebug("applying trajopt plan profile to mixed waypoint for index %d", index);
  size_t joint_num = mixed_waypoint.getJointNames().size();
  assert(!(manip_info.empty() && parent_instruction.getManipulatorInfo().empty()));
  auto mi = manip_info.getCombined(parent_instruction.getManipulatorInfo());
  Eigen::Isometry3d tcp_offset = pci.env->findTCPOffset(mi);

  if (!is_target)
  {
    for (auto t : mixed_waypoint.getLinkConstraints())
    {
      auto cart_coeff = t.second.coeff.size() ? t.second.coeff : cartesian_coeff;
      auto ti_cartesian = createCartesianWaypointTermInfo(
          index, mi.working_frame, t.second.pose, t.first, tcp_offset, cart_coeff, trajopt::TermType::TT_CNT);
      pci.cnt_infos.push_back(ti_cartesian);
    }
    return;
  }

  // generate waypoint and coeff from target joint values
  Eigen::VectorXd adjusted_joint_coeff = Eigen::VectorXd::Zero(joint_num);
  std::vector<std::string> joint_names = mixed_waypoint.getJointNames();
  JointWaypoint joint_waypoint(joint_names, Eigen::VectorXd::Zero(joint_num));
  for (auto t : mixed_waypoint.getJointTargets())
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), t.first);
    if (it == joint_names.end())
    {
      throw std::logic_error("cannot find target joint name");
    }
    int idx = std::distance(joint_names.begin(), it);

    assert(idx < joint_num);
    adjusted_joint_coeff(idx) = joint_coeff.size() == adjusted_joint_coeff.size() ?
                                    joint_coeff(idx) :
                                    joint_coeff(0);  // fixed joint coeff size
    joint_waypoint.getPosition()(idx) = t.second;
  }

  // CONSOLE_BRIDGE_logDebug("creating joint and cartesian terms");

  auto ti_joint = createJointWaypointTermInfo(joint_waypoint.getPosition(), index, adjusted_joint_coeff, term_type);
  term_infos.push_back(ti_joint);

  // build cartesian term info

  for (auto& t : mixed_waypoint.getLinkTargets())
  {
    auto ti_cartesian = createCartesianWaypointTermInfo(
        index, mi.working_frame, t.second, t.first, tcp_offset, cartesian_coeff, term_type);
    term_infos.push_back(ti_cartesian);
  }

  for (auto ti : term_infos)
  {
    if (term_type == trajopt::TermType::TT_CNT)
      pci.cnt_infos.push_back(ti);
    else
      pci.cost_infos.push_back(ti);
  }
  // Add constraints from error functions if available.
  addConstraintErrorFunctions(pci, index);
}

void TrajOptDefaultPlanProfile::addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, int index) const
{
  if (fixed_dofs.size())
  {
    pci.basic_info.fixed_dofs = fixed_dofs;
  }
  for (const auto& c : constraint_error_functions)
  {
    trajopt::TermInfo::Ptr ti =
        createUserDefinedTermInfo(index, index, std::get<0>(c), std::get<1>(c), trajopt::TT_CNT);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
    ef->constraint_type = std::get<2>(c);
    ef->coeff = std::get<3>(c);

    pci.cnt_infos.push_back(ef);
  }
}

tinyxml2::XMLElement* TrajOptDefaultPlanProfile::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  tinyxml2::XMLElement* xml_planner = doc.NewElement("Planner");
  xml_planner->SetAttribute("type", std::to_string(1).c_str());

  tinyxml2::XMLElement* xml_trajopt = doc.NewElement("TrajOptDefaultPlanProfile");

  tinyxml2::XMLElement* xml_cart_coeff = doc.NewElement("CartesianCoefficients");
  std::stringstream cart_coeff;
  cart_coeff << cartesian_coeff.format(eigen_format);
  xml_cart_coeff->SetText(cart_coeff.str().c_str());
  xml_trajopt->InsertEndChild(xml_cart_coeff);

  tinyxml2::XMLElement* xml_joint_coeff = doc.NewElement("JointCoefficients");
  std::stringstream jnt_coeff;
  jnt_coeff << joint_coeff.format(eigen_format);
  xml_joint_coeff->SetText(jnt_coeff.str().c_str());
  xml_trajopt->InsertEndChild(xml_joint_coeff);

  tinyxml2::XMLElement* xml_term_type = doc.NewElement("Term");
  xml_term_type->SetAttribute("type", static_cast<int>(term_type));
  xml_trajopt->InsertEndChild(xml_term_type);

  xml_planner->InsertEndChild(xml_trajopt);

  return xml_planner;
}
}  // namespace tesseract_planning
