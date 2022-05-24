#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract_environment/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/3mo/3mo_motion_planner.h>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>

namespace tesseract_planning
{
MMMOMotionPlannerStatusCategory::MMMOMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& MMMOMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string MMMOMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound: {
      return "Found valid solution";
    }
    case ErrorInvalidInput: {
      return "Input to planner is invalid. Check that instructions and seed are compatible";
    }
    case FailedToFindValidSolution: {
      return "Failed to find valid solution";
    }
    default: {
      assert(false);
      return "";
    }
  }
}

MMMOMotionPlanner::MMMOMotionPlanner(std::string name)
  : name_(std::move(name)), status_category_(std::make_shared<const MMMOMotionPlannerStatusCategory>(name_))
{
  if (name_.empty())
    throw std::runtime_error("MMMOMotionPlanner name is empty!");
}

const std::string& MMMOMotionPlanner::getName() const { return name_; }

bool MMMOMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing planning is not implemented yet");
  return false;
}

void MMMOMotionPlanner::clear() {}

MotionPlanner::Ptr MMMOMotionPlanner::clone() const { return std::make_shared<MMMOMotionPlanner>(name_); }

tesseract_common::StatusCode MMMOMotionPlanner::solve(const PlannerRequest& request,
                                                      PlannerResponse& response,
                                                      bool /*verbose*/) const
{
  if (!checkUserInput(request))
  {
    response.status =
        tesseract_common::StatusCode(MMMOMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Assume all the plan instructions have the same manipulator as the composite
  const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
  const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;

  // Initialize
  tesseract_kinematics::JointGroup::UPtr manip = request.env->getJointGroup(manipulator);
  Waypoint start_waypoint{ NullWaypoint() };

  // Create seed
  CompositeInstruction seed;

  // Get the start waypoint/instruction
  PlanInstruction start_instruction = getStartInstruction(request, request.env_state, *manip);

  // Set start instruction
  MoveInstruction start_instruction_seed(start_instruction.getWaypoint(), start_instruction);
  start_instruction_seed.setMoveType(MoveInstructionType::START);

  // Process the instructions into the seed
  try
  {
    PlanInstruction start_instruction_copy = start_instruction;
    MoveInstruction start_instruction_seed_copy = start_instruction_seed;
    seed =
        processCompositeInstruction(request.instructions, start_instruction_copy, start_instruction_seed_copy, request);
  }
  catch (std::exception& e)
  {
    CONSOLE_BRIDGE_logError("MMMOMotionPlanner failed to generate problem: %s.", e.what());
    response.status =
        tesseract_common::StatusCode(MMMOMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Set seed start state
  seed.setStartInstruction(start_instruction_seed);

  // Fill out the response
  response.results = seed;

  // Enforce limits
  auto results_flattened = flatten(response.results, &moveFilter);
  for (auto& inst : results_flattened)
  {
    auto& mi = inst.get().as<MoveInstruction>();
    Eigen::VectorXd jp = getJointPosition(mi.getWaypoint());
    assert(tesseract_common::satisfiesPositionLimits(jp, manip->getLimits().joint_limits));
    tesseract_common::enforcePositionLimits(jp, manip->getLimits().joint_limits);
    setJointPosition(mi.getWaypoint(), jp);
  }

  // Return success
  response.status = tesseract_common::StatusCode(MMMOMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

PlanInstruction MMMOMotionPlanner::getStartInstruction(const PlannerRequest& request,
                                                       const tesseract_scene_graph::SceneState& current_state,
                                                       const tesseract_kinematics::JointGroup& manip)
{
  // Create start instruction
  Waypoint start_waypoint{ NullWaypoint() };
  PlanInstruction start_instruction_seed(start_waypoint, PlanInstructionType::START);

  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    const auto& start_instruction = request.instructions.getStartInstruction().as<PlanInstruction>();
    assert(start_instruction.isStart());
    start_waypoint = start_instruction.getWaypoint();

    if (isJointWaypoint(start_waypoint))
    {
      assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
      const auto& jwp = start_waypoint.as<JointWaypoint>();
      start_instruction_seed.setWaypoint(StateWaypoint(jwp.joint_names, jwp.waypoint));
    }
    else if (isCartesianWaypoint(start_waypoint))
    {
      /** @todo Update to run IK to find solution closest to start */
      StateWaypoint temp(manip.getJointNames(), current_state.getJointValues(manip.getJointNames()));
      start_waypoint = temp;

      start_instruction_seed.setWaypoint(start_waypoint);
    }
    else if (isStateWaypoint(start_waypoint))
    {
      assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
      start_instruction_seed.setWaypoint(start_waypoint);
    }
    else
    {
      throw std::runtime_error("Unsupported waypoint type!");
    }
    start_instruction_seed.setDescription(start_instruction.getDescription());
    start_instruction_seed.setProfile(start_instruction.getProfile());
    start_instruction_seed.profile_overrides = start_instruction.profile_overrides;
    start_instruction_seed.setManipulatorInfo(start_instruction.getManipulatorInfo());
  }
  else
  {
    StateWaypoint temp(manip.getJointNames(), current_state.getJointValues(manip.getJointNames()));
    start_waypoint = temp;

    start_instruction_seed.setWaypoint(start_waypoint);
  }

  return start_instruction_seed;
}

CompositeInstruction MMMOMotionPlanner::processCompositeInstruction(const CompositeInstruction& instructions,
                                                                    PlanInstruction& prev_instruction,
                                                                    MoveInstruction& prev_seed,
                                                                    const PlannerRequest& request) const
{
  CompositeInstruction seed(instructions);
  seed.clear();

  for (std::size_t i = 0; i < instructions.size(); ++i)
  {
    const auto& instruction = instructions[i];

    if (isCompositeInstruction(instruction))
    {
      seed.push_back(
          processCompositeInstruction(instruction.as<CompositeInstruction>(), prev_instruction, prev_seed, request));
    }
    else if (isPlanInstruction(instruction))
    {
      const auto& base_instruction = instruction.as<PlanInstruction>();

      // Get the next plan instruction if it exists
      Instruction next_instruction = NullInstruction();
      for (std::size_t n = i + 1; n < instructions.size(); ++n)
      {
        if (isPlanInstruction(instructions[n]))
        {
          next_instruction = instructions[n];
          break;
        }
      }

      // If a path profile exists for the instruction it should use that instead of the termination profile
      MMMOPlannerPlanProfile::ConstPtr plan_profile;
      MapInfo default_map = MapInfo();
      if (base_instruction.getPathProfile().empty())
      {
        std::string profile = getProfileString(name_, base_instruction.getProfile(), request.plan_profile_remapping);
        plan_profile = getProfile<MMMOPlannerPlanProfile>(
            name_,
            profile,
            *request.profiles,
            std::make_shared<MMMOPlannerPlanProfile>(default_map.map_x, default_map.map_y, default_map.step_size));
        plan_profile = applyProfileOverrides(name_, profile, plan_profile, base_instruction.profile_overrides);
      }
      else
      {
        std::string profile =
            getProfileString(name_, base_instruction.getPathProfile(), request.plan_profile_remapping);
        plan_profile = getProfile<MMMOPlannerPlanProfile>(
            name_,
            profile,
            *request.profiles,
            std::make_shared<MMMOPlannerPlanProfile>(default_map.map_x, default_map.map_y, default_map.step_size));
        plan_profile = applyProfileOverrides(name_, profile, plan_profile, base_instruction.profile_overrides);
      }

      if (!plan_profile)
        throw std::runtime_error("MMMOMotionPlanner: Invalid profile");

      CompositeInstruction instruction_seed = plan_profile->generate(prev_instruction,
                                                                     prev_seed,
                                                                     base_instruction,
                                                                     next_instruction,
                                                                     request,
                                                                     request.instructions.getManipulatorInfo());
      seed.push_back(instruction_seed);

      prev_instruction = base_instruction;
      prev_seed = instruction_seed.back().as<MoveInstruction>();
    }
    else if (isMoveInstruction(instruction))
    {
      throw std::runtime_error("MMMOMotionPlanner: The input program includes MoveInstructions!");
    }
    else
    {
      seed.push_back(instruction);
    }
  }  // for (const auto& instruction : instructions)
  return seed;
}

bool MMMOMotionPlanner::checkUserInput(const PlannerRequest& request)
{
  // Check that parameters are valid
  if (request.env == nullptr)
  {
    CONSOLE_BRIDGE_logError("In MMMOMotionPlanner: env is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("MMMOMotionPlanner requires at least one instruction");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
