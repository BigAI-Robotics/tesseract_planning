/**
 * @file ompl_motion_planner_task.h
 * @brief OMPL motion planning pipeline
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/ompl_motion_pipeline_task.h>

#include <tesseract_task_composer/nodes/motion_planner_task.h>
#include <tesseract_task_composer/nodes/has_seed_task.h>
#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/nodes/ruckig_trajectory_smoothing_task.h>
#include <tesseract_task_composer/nodes/check_input_task.h>
#include <tesseract_task_composer/nodes/done_task.h>
#include <tesseract_task_composer/nodes/error_task.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

namespace tesseract_planning
{
OMPLMotionPipelineTask::OMPLMotionPipelineTask(std::string name) : TaskComposerGraph(name)
{
  ctor(uuid_str_, uuid_str_);
}

OMPLMotionPipelineTask::OMPLMotionPipelineTask(std::string input_key, std::string output_key, std::string name)
  : TaskComposerGraph(name)
{
  ctor(std::move(input_key), std::move(output_key));
}
OMPLMotionPipelineTask::OMPLMotionPipelineTask(std::string input_key,
                                               std::string output_key,
                                               bool check_input,
                                               bool post_collision_check,
                                               bool post_smoothing,
                                               std::string name)
  : TaskComposerGraph(name)
  , check_input_(check_input)
  , post_collision_check_(post_collision_check)
  , post_smoothing_(post_smoothing)
{
  ctor(std::move(input_key), std::move(output_key));
}

void OMPLMotionPipelineTask::ctor(std::string input_key, std::string output_key)
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));

  boost::uuids::uuid done_task = addNode(std::make_unique<DoneTask>());
  boost::uuids::uuid error_task = addNode(std::make_unique<ErrorTask>());

  boost::uuids::uuid check_input_task;
  if (check_input_)
    check_input_task = addNode(std::make_unique<CheckInputTask>(input_keys_[0]));

  // Check if seed was provided
  boost::uuids::uuid has_seed_task = addNode(std::make_unique<HasSeedTask>());

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  boost::uuids::uuid interpolator_task =
      addNode(std::make_unique<MotionPlannerTask>(interpolator, input_keys_[0], output_keys_[0]));

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  boost::uuids::uuid seed_min_length_task = addNode(std::make_unique<MinLengthTask>(output_keys_[0], output_keys_[0]));

  // Setup TrajOpt
  auto motion_planner = std::make_shared<OMPLMotionPlanner>();
  boost::uuids::uuid motion_planner_task =
      addNode(std::make_unique<MotionPlannerTask>(motion_planner, output_keys_[0], output_keys_[0], false));

  // Setup post collision check
  boost::uuids::uuid contact_check_task;
  if (post_collision_check_)
    contact_check_task = addNode(std::make_unique<DiscreteContactCheckTask>(output_keys_[0]));

  // Setup time parameterization
  boost::uuids::uuid time_parameterization_task =
      addNode(std::make_unique<IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

  // Setup trajectory smoothing
  boost::uuids::uuid smoothing_task;
  if (post_smoothing_)
    smoothing_task = addNode(std::make_unique<RuckigTrajectorySmoothingTask>(output_keys_[0], output_keys_[0]));

  if (check_input_)
    addEdges(check_input_task, { error_task, has_seed_task });

  addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  addEdges(interpolator_task, { error_task, seed_min_length_task });
  addEdges(seed_min_length_task, { motion_planner_task });

  if (post_collision_check_)
  {
    addEdges(motion_planner_task, { error_task, contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
  }
  else
  {
    addEdges(motion_planner_task, { error_task, time_parameterization_task });
  }

  if (post_smoothing_)
  {
    addEdges(time_parameterization_task, { error_task, smoothing_task });
    addEdges(smoothing_task, { done_task });
  }
  else
  {
    addEdges(time_parameterization_task, { error_task, done_task });
  }
}

TaskComposerNode::UPtr OMPLMotionPipelineTask::clone() const
{
  return std::make_unique<OMPLMotionPipelineTask>(
      input_keys_[0], output_keys_[0], check_input_, post_collision_check_, post_smoothing_, name_);
}

bool OMPLMotionPipelineTask::operator==(const OMPLMotionPipelineTask& rhs) const
{
  bool equal = true;
  equal &= (check_input_ == rhs.check_input_);
  equal &= (post_collision_check_ == rhs.post_collision_check_);
  equal &= (post_smoothing_ == rhs.post_smoothing_);
  equal &= TaskComposerGraph::operator==(rhs);
  return equal;
}
bool OMPLMotionPipelineTask::operator!=(const OMPLMotionPipelineTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void OMPLMotionPipelineTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(check_input_);
  ar& BOOST_SERIALIZATION_NVP(post_collision_check_);
  ar& BOOST_SERIALIZATION_NVP(post_smoothing_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::OMPLMotionPipelineTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::OMPLMotionPipelineTask)
