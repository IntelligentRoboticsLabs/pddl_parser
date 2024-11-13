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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/State.hpp"

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "plansys2_pddl_parser/Domain.hpp"

#include "plansys2_domain_expert/DomainExpertInterface.hpp"
#include "plansys2_domain_expert/DomainReader.hpp"

namespace plansys2
{

/// DomainExpert is in charge of managing the internal structure of a domain.
class DomainExpert : public DomainExpertInterface
{
public:
  /// Create a new DomainExpert with the content of a domain.
  /**
   * \param[in] node_name The content of a PDDL domain.
   */
  explicit DomainExpert(const std::string & domain);

  /// Extend the content of aDomainExpert with the content of another domain.
  /**
   * \param[in] node_name The content of a PDDL domain.
   */
  void extendDomain(const std::string & domain);

  /// Get the domain name.
  /**
   * \return A string containing the domain name.
   */
  std::string getName();

  /// Get the types existing in the domain.
  /**
   * \return The vector containing the names of the types.
   */
  std::vector<std::string> getTypes();

  /// Get the details of a constants existing for a type.
  /**
   * \param[in] predicate The name of the type.
   * \return A list of constants names for the passed type
   */
  std::vector<std::string> getConstants(const std::string & type);

  /// Get the predicates existing in the domain.
  /**
   * \return The vector containing the name of the predicates.
   */
  std::vector<plansys2::Predicate> getPredicates();

  /// Get the details of a predicate existing in the domain.
  /**
   * \param[in] predicate The name of the predicate.
   * \return A Predicate object containing the predicate name and its parameters (name and type).
   *    If the predicate does not exist, the value returned has not value.
   */
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  /// Get the functions existing in the domain.
  /**
   * \return The vector containing the name of the functions.
   */
  std::vector<plansys2::Function> getFunctions();

  /// Get the details of a function existing in the domain.
  /**
   * \param[in] function The name of the function.
   * \return A Function object containing the function name and its parameters (name and type).
   *    If the function does not exist, the value returned has not value.
   */
  std::optional<plansys2::Function> getFunction(const std::string & function);

  plansys2_msgs::msg::Derived getDerivedFromDomain(
    unsigned int i, const std::vector<std::string> & params = {});

  /// Get the derived predicates existing in the domain.
  /**
   * \return The vector containing the derived predicates.
   */
  std::vector<plansys2_msgs::msg::Derived> getDerivedPredicates();

  plansys2::DerivedGraph getDerivedPredicatesGraph();

  /// Get the details of a derived predicate existing in the domain.
  /**
   * \param[in] predicate The name of the predicate.
   * \return A Derived object containing the predicate name, its parameters (name and type), and preconditions.
   *    If the predicate does not exist, the value returned has not value.
   */
  std::vector<plansys2_msgs::msg::Derived> getDerivedPredicate(
    const std::string & predicate,
    const std::vector<std::string> & params = {});

  /// Get the regular actions existing in the domain.
  /**
   * \return The vector containing the names of the actions.
   */
  std::vector<std::string> getActions();

  /// Get the details of an regular action existing in the domain.
  /**
   * \param[in] action The name of the action.
   * \return An Action object containing the action name, parameters, requirements and effects.
   *    If the action does not exist, the value returned has not value.
   */
  plansys2_msgs::msg::Action::SharedPtr getAction(
    const std::string & action,
    const std::vector<std::string> & params = {});

  /// Get the temporal actions existing in the domain.
  /**
   * \return The vector containing the names of the actions.
   */
  std::vector<std::string> getDurativeActions();

  /// Get the details of an durative action existing in the domain.
  /**
   * \param[in] action The name of the action.
   * \return A Durative Action object containing the action name, parameters, requirements and
   *    effects. If the action does not exist, the value returned has not value.
   */
  plansys2_msgs::msg::DurativeAction::SharedPtr getDurativeAction(
    const std::string & action,
    const std::vector<std::string> & params = {});

  /// Get the current domain, ready to be saved to file, or to initialize another domain.
  /**
   * \return A string containing the domain.
   */
  std::string getDomain();

  /// Determine if a particular domain exists.
  /**
   * \param[in] domain The name of the domain.
   * \return true if the domain exists.
   */
  bool existDomain(const std::string & domain_name);

private:
  std::shared_ptr<parser::pddl::Domain> domain_;
  DomainReader domains_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
