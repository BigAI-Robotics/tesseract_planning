/**
 * @file composite_instruction.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_common/manipulator_info.h>

namespace tesseract_planning
{
class CompositeInstruction;

/**
 * @brief This is used for filtering only what you want in the vector
 *
 * The first parameter is the instruction consider, the second is it's parent composite instruction, and the third
 * indicates if the parent composite is the top most composite
 *
 * The filter should return true when the instruction passed should be included not throw.
 */
using flattenFilterFn =
    std::function<bool(const InstructionPoly&, const CompositeInstruction&, bool parent_is_first_composite)>;
using locateFilterFn =
    std::function<bool(const InstructionPoly&, const CompositeInstruction&, bool parent_is_first_composite)>;

bool moveFilter(const InstructionPoly& instruction,
                const CompositeInstruction& composite,
                bool parent_is_first_composite);

enum class CompositeInstructionOrder
{
  ORDERED,               // Must go in forward
  UNORDERED,             // Any order is allowed
  ORDERED_AND_REVERABLE  // Can go forward or reverse the order
};

class CompositeInstruction
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CompositeInstruction(std::string profile = DEFAULT_PROFILE_KEY,
                       CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED,
                       tesseract_common::ManipulatorInfo manipulator_info = tesseract_common::ManipulatorInfo());

  CompositeInstructionOrder getOrder() const;

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void print(const std::string& prefix = "") const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  void setProfileOverrides(ProfileDictionary::ConstPtr profile_overrides);
  ProfileDictionary::ConstPtr getProfileOverrides() const;

  void setManipulatorInfo(tesseract_common::ManipulatorInfo info);
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const;
  tesseract_common::ManipulatorInfo& getManipulatorInfo();

  void setStartInstruction(MoveInstructionPoly instruction);

  void resetStartInstruction();
  const MoveInstructionPoly& getStartInstruction() const;
  MoveInstructionPoly& getStartInstruction();
  bool hasStartInstruction() const;

  void setInstructions(std::vector<tesseract_planning::InstructionPoly> instructions);
  std::vector<tesseract_planning::InstructionPoly>& getInstructions();
  const std::vector<tesseract_planning::InstructionPoly>& getInstructions() const;

  void appendMoveInstruction(const MoveInstructionPoly& mi);
  void appendMoveInstruction(const MoveInstructionPoly&& mi);

  void appendInstruction(const InstructionPoly& i);
  void appendInstruction(const InstructionPoly&& i);

  /**
   * @brief Get the first Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The first Move Instruction (Non-Const)
   */
  MoveInstructionPoly* getFirstMoveInstruction();

  /**
   * @brief Get the first Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The first Move Instruction (Const)
   */
  const MoveInstructionPoly* getFirstMoveInstruction() const;

  /**
   * @brief Get the last Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The last Move Instruction (Non-Const)
   */
  MoveInstructionPoly* getLastMoveInstruction();

  /**
   * @brief Get the last Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The last Move Instruction (Const)
   */
  const MoveInstructionPoly* getLastMoveInstruction() const;

  /**
   * @brief Get number of Move Instruction in a Composite Instruction
   * This does not consider the start instruction in the child composite instruction
   * @param composite_instruction The Composite Instruction to process
   * @return The number of Move Instructions
   */
  long getMoveInstructionCount() const;

  /**
   * @brief Get the first Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The first Instruction (Const)
   */
  const InstructionPoly* getFirstInstruction(const locateFilterFn& locate_filter = nullptr,
                                             bool process_child_composites = true) const;

  /**
   * @brief Get the first Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The first Instruction (Non-Const)
   */
  InstructionPoly* getFirstInstruction(const locateFilterFn& locate_filter = nullptr,
                                       bool process_child_composites = true);

  /**
   * @brief Get the last Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The Last Instruction (Const)
   */
  const InstructionPoly* getLastInstruction(const locateFilterFn& locate_filter = nullptr,
                                            bool process_child_composites = true) const;

  /**
   * @brief Get the last Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The Last Instruction (Non-Const)
   */
  InstructionPoly* getLastInstruction(const locateFilterFn& locate_filter = nullptr,
                                      bool process_child_composites = true);

  /**
   * @brief Get number of Instruction in a Composite Instruction
   * @param composite_instruction The Composite Instruction to process
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The number of Instructions
   */
  long getInstructionCount(const locateFilterFn& locate_filter = nullptr, bool process_child_composites = true) const;

  /**
   * @brief Flattens a CompositeInstruction into a vector of Instruction
   * @param composite_instruction Input composite instruction to be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @return A new flattened vector referencing the original instruction elements
   */
  std::vector<std::reference_wrapper<InstructionPoly>> flatten(const flattenFilterFn& filter = nullptr);

  /**
   * @brief Flattens a CompositeInstruction into a vector of Instruction&
   * @param instruction Input composite instruction to be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @return A new flattened vector referencing the original instruction elements
   */
  std::vector<std::reference_wrapper<const InstructionPoly>> flatten(const flattenFilterFn& filter = nullptr) const;

  /**
   * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
   * instruction will only be flattened if the corresponding element in pattern is flattenable.
   * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to
   * composites in the parent instruction
   * @param instruction CompositeInstruction that will be flattened
   * @param pattern CompositeInstruction used to determine if instruction will be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @return A new flattened vector referencing the original instruction elements
   */
  std::vector<std::reference_wrapper<InstructionPoly>> flattenToPattern(const CompositeInstruction& pattern,
                                                                        const flattenFilterFn& filter = nullptr);
  /**
   * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
   * instruction will only be flattened if the corresponding element in pattern is flattenable.
   * @param instruction CompositeInstruction that will be flattened
   * @param pattern CompositeInstruction used to determine if instruction will be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @return A new flattened vector referencing the original instruction elements
   */
  std::vector<std::reference_wrapper<const InstructionPoly>>
  flattenToPattern(const CompositeInstruction& pattern, const flattenFilterFn& filter = nullptr) const;

  bool operator==(const CompositeInstruction& rhs) const;

  bool operator!=(const CompositeInstruction& rhs) const;

  // C++ container support

  /** value_type */
  using value_type = InstructionPoly;
  /** pointer */
  using pointer = typename std::vector<value_type>::pointer;
  /** const_pointer */
  using const_pointer = typename std::vector<value_type>::const_pointer;
  /** reference */
  using reference = typename std::vector<value_type>::reference;
  /** const_reference */
  using const_reference = typename std::vector<value_type>::const_reference;
  /** size_type */
  using size_type = typename std::vector<value_type>::size_type;
  /** difference_type */
  using difference_type = typename std::vector<value_type>::difference_type;
  /** iterator */
  using iterator = typename std::vector<value_type>::iterator;
  /** const_iterator */
  using const_iterator = typename std::vector<value_type>::const_iterator;
  /** reverse_iterator */
  using reverse_iterator = typename std::vector<value_type>::reverse_iterator;
  /** const_reverse_iterator */
  using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;

  //  template <class InputIt>
  //  CompositeInstruction(InputIt first, InputIt last) : container_(first, last)
  //  {
  //  }

  ///////////////
  // Iterators //
  ///////////////
  /** @brief returns an iterator to the beginning */
  iterator begin();
  /** @brief returns an iterator to the beginning */
  const_iterator begin() const;
  /** @brief returns an iterator to the end */
  iterator end();
  /** @brief returns an iterator to the end */
  const_iterator end() const;
  /** @brief returns a reverse iterator to the beginning */
  reverse_iterator rbegin();
  /** @brief returns a reverse iterator to the beginning */
  const_reverse_iterator rbegin() const;
  /** @brief returns a reverse iterator to the end */
  reverse_iterator rend();
  /** @brief returns a reverse iterator to the end */
  const_reverse_iterator rend() const;
  /** @brief returns an iterator to the beginning */
  const_iterator cbegin() const;
  /** @brief returns an iterator to the end */
  const_iterator cend() const;
  /** @brief returns a reverse iterator to the beginning */
  const_reverse_iterator crbegin() const;
  /** @brief returns a reverse iterator to the end */
  const_reverse_iterator crend() const;

  //////////////
  // Capacity //
  //////////////
  /** @brief checks whether the container is empty */
  bool empty() const;
  /** @brief returns the number of elements */
  size_type size() const;
  /** @brief returns the maximum possible number of elements */
  size_type max_size() const;
  /** @brief reserve number of elements */
  void reserve(size_type n);
  /** @brief returns the number of elements that can be held in currently allocated storage */
  size_type capacity() const;
  /** @brief reduces memory usage by freeing unused memory  */
  void shrink_to_fit();

  ////////////////////
  // Element Access //
  ////////////////////
  /** @brief access the first element */
  reference front();
  /** @brief access the first element */
  const_reference front() const;
  /** @brief access the last element */
  reference back();
  /** @brief access the last element */
  const_reference back() const;
  /** @brief access specified element with bounds checking */
  reference at(size_type n);
  /** @brief access specified element with bounds checking */
  const_reference at(size_type n) const;
  /** @brief direct access to the underlying array  */
  pointer data();
  /** @brief direct access to the underlying array  */
  const_pointer data() const;
  /** @brief access specified element */
  reference operator[](size_type pos);
  /** @brief access specified element */
  const_reference operator[](size_type pos) const;

  ///////////////
  // Modifiers //
  ///////////////
  /** @brief clears the contents */
  void clear();

  /** @brief erases element */
  iterator erase(const_iterator p);
  iterator erase(const_iterator first, const_iterator last);

  /** @brief removes the last element */
  void pop_back();

  /** @brief swaps the contents  */
  void swap(std::vector<value_type>& other);

private:
  std::vector<InstructionPoly> container_;

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Composite Instruction" };

  /** @brief Contains information about the manipulator associated with this instruction*/
  tesseract_common::ManipulatorInfo manipulator_info_;

  /**
   * @brief The profile applied its child plan instructions
   *
   * If it has a child composite instruction it uses the child composites profile for that section
   */
  std::string profile_{ DEFAULT_PROFILE_KEY };

  /** @brief Dictionary of profiles that will override named profiles for a specific task*/
  ProfileDictionary::ConstPtr profile_overrides_;

  /** @brief The order of the composite instruction */
  CompositeInstructionOrder order_{ CompositeInstructionOrder::ORDERED };

  /**
   * @brief The start instruction to use for composite instruction. This should be of type
   * MoveInstruction but is stored as type Instruction because it is not required
   *
   * If not provided, the planner should use the current state of the robot is used and defined as fixed.
   */
  InstructionPoly start_instruction_;

  const InstructionPoly* getFirstInstructionHelper(const CompositeInstruction& composite_instruction,
                                                   const locateFilterFn& locate_filter,
                                                   bool process_child_composites,
                                                   bool first_composite) const;

  InstructionPoly* getFirstInstructionHelper(CompositeInstruction& composite_instruction,
                                             const locateFilterFn& locate_filter,
                                             bool process_child_composites,
                                             bool first_composite);

  const InstructionPoly* getLastInstructionHelper(const CompositeInstruction& composite_instruction,
                                                  const locateFilterFn& locate_filter,
                                                  bool process_child_composites,
                                                  bool first_composite) const;

  InstructionPoly* getLastInstructionHelper(CompositeInstruction& composite_instruction,
                                            const locateFilterFn& locate_filter,
                                            bool process_child_composites,
                                            bool first_composite);

  long getInstructionCountHelper(const CompositeInstruction& composite_instruction,
                                 const locateFilterFn& locate_filter,
                                 bool process_child_composites,
                                 bool first_composite) const;

  /**
   * @brief Helper function used by Flatten. Not intended for direct use
   * @param flattened Vector of instructions representing the full flattened composite
   * @param composite Composite instruction to be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @param first_composite Indicates if the composite being processed is the top most composite
   */
  void flattenHelper(std::vector<std::reference_wrapper<InstructionPoly>>& flattened,
                     CompositeInstruction& composite,
                     const flattenFilterFn& filter,
                     bool first_composite);

  /**
   * @brief Helper function used by Flatten. Not intended for direct use
   * @param flattened Vector of instructions representing the full flattened composite
   * @param composite Composite instruction to be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @param first_composite Indicates if the composite being processed is the top most composite
   */
  void flattenHelper(std::vector<std::reference_wrapper<const InstructionPoly>>& flattened,
                     const CompositeInstruction& composite,
                     const flattenFilterFn& filter,
                     bool first_composite) const;

  /**
   * @brief Helper function used by FlattenToPattern. Not intended for direct use
   * @param flattened Vector of instructions representing the full flattened composite
   * @param composite Composite instruction to be flattened
   * @param pattern CompositeInstruction used to determine if instruction will be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @param first_composite Indicates if the composite being processed is the top most composite
   */
  void flattenToPatternHelper(std::vector<std::reference_wrapper<InstructionPoly>>& flattened,
                              CompositeInstruction& composite,
                              const CompositeInstruction& pattern,
                              const flattenFilterFn& filter,
                              bool first_composite);

  /**
   * @brief Helper function used by FlattenToPattern. Not intended for direct use
   * @param flattened Vector of instructions representing the full flattened composite
   * @param composite Composite instruction to be flattened
   * @param pattern CompositeInstruction used to determine if instruction will be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @param first_composite Indicates if the composite being processed is the top most composite
   */
  void flattenToPatternHelper(std::vector<std::reference_wrapper<const InstructionPoly>>& flattened,
                              const CompositeInstruction& composite,
                              const CompositeInstruction& pattern,
                              const flattenFilterFn& filter,
                              bool first_composite) const;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

TESSERACT_INSTRUCTION_EXPORT_KEY(tesseract_planning, CompositeInstruction);

#endif  // TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
