// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "plansys2_core/Types.hpp"

namespace plansys2
{

/// DomainExpertInterface is the interface for both DomainExpert and DomainExpertClient
class DomainExpertInterface
{
public:
  /// Void constructor
  /**
   */
  DomainExpertInterface() {}

  /// Get the domain name.
  /**
   * \return A string containing the domain name.
   */
  virtual std::string getName() = 0;

  /// Get the types existing in the domain.
  /**
   * \return The vector containing the names of the types.
   */
  virtual std::vector<std::string> getTypes() = 0;

  /// Get the details of a constants existing for a type.
  /**
   * \param[in] predicate The name of the type.
   * \return A list of constants names for the passed type
   */
  virtual std::vector<std::string> getConstants(const std::string & type) = 0;

  /// Get the predicates existing in the domain.
  /**
   * \return The vector containing the name of the predicates.
   */
  virtual std::vector<plansys2::Predicate> getPredicates() = 0;

  /// Get the details of a predicate existing in the domain.
  /**
   * \param[in] predicate The name of the predicate.
   * \return A Predicate object containing the predicate name and its parameters (name and type).
   *    If the predicate does not exist, the value returned has not value.
   */
  virtual std::optional<plansys2::Predicate> getPredicate(const std::string & predicate)
  = 0;

  /// Get the functions existing in the domain.
  /**
   * \return The vector containing the name of the functions.
   */
  virtual std::vector<plansys2::Function> getFunctions() = 0;

  /// Get the details of a function existing in the domain.
  /**
   * \param[in] function The name of the function.
   * \return A Function object containing the function name and its parameters (name and type).
   *    If the function does not exist, the value returned has not value.
   */
  virtual std::optional<plansys2::Function> getFunction(const std::string & function) =
  0;

  /// Get the derived predicates existing in the domain.
  /**
   * \return The vector containing the derived predicates.
   */
  virtual std::vector<plansys2_msgs::msg::Derived> getDerivedPredicates() = 0;

  /// Get the details of a derived predicate existing in the domain.
  /**
   * \param[in] predicate The name of the predicate.
   * \return A Derived object containing the predicate name, its parameters (name and type), and preconditions.
   *    If the predicate does not exist, the value returned has not value.
   */
  virtual std::vector<plansys2_msgs::msg::Derived> getDerivedPredicate(
    const std::string & predicate,
    const std::vector<std::string> & params = {}) = 0;

  /// Get the regular actions existing in the domain.
  /**
   * \return The vector containing the names of the actions.
   */
  virtual std::vector<std::string> getActions() = 0;

  /// Get the details of an regular action existing in the domain.
  /**
   * \param[in] action The name of the action.
   * \return An Action object containing the action name, parameters, requirements and effects.
   *    If the action does not exist, the value returned has not value.
   */
  virtual plansys2_msgs::msg::Action::SharedPtr getAction(
    const std::string & action, const std::vector<std::string> & params) =
  0;

  /// Get the temporal actions existing in the domain.
  /**
   * \return The vector containing the names of the actions.
   */
  virtual std::vector<std::string> getDurativeActions() = 0;

  /// Get the details of an durative action existing in the domain.
  /**
   * \param[in] action The name of the action.
   * \return A Durative Action object containing the action name, parameters, requirements and
   *    effects. If the action does not exist, the value returned has not value.
   */
  virtual plansys2_msgs::msg::DurativeAction::SharedPtr getDurativeAction(
    const std::string & durative_action, const std::vector<std::string> & params) =
  0;

  /// Get the current domain, ready to be saved to file, or to initialize another domain.
  /**
   * \return A string containing the domain.
   */
  virtual std::string getDomain() = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_
