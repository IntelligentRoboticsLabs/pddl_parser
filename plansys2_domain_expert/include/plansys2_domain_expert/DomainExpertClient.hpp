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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/Types.hpp"
#include "plansys2_domain_expert/DomainExpertInterface.hpp"

#include "std_msgs/msg/string.hpp"

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "plansys2_msgs/srv/get_domain.hpp"
#include "plansys2_msgs/srv/get_domain_name.hpp"
#include "plansys2_msgs/srv/get_domain_types.hpp"
#include "plansys2_msgs/srv/get_domain_constants.hpp"
#include "plansys2_msgs/srv/get_domain_actions.hpp"
#include "plansys2_msgs/srv/get_domain_action_details.hpp"
#include "plansys2_msgs/srv/get_domain_derived_predicate_details.hpp"
#include "plansys2_msgs/srv/get_domain_durative_action_details.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/// DomainExpertClient requests changes or gets information to/from the DomainExpertNode
/**
 * Any node can create a DomainExpertClient object to requests changes to the
 * DomainExpertNode, or to get information from it. It presents the same interface
 * of the DomainExpert, and hides the complexity of using services.
 */
class DomainExpertClient : public DomainExpertInterface
{
public:
  /// Create a new DomainExpertClient.
  DomainExpertClient();

  /// Get the domain name.
  /**
   * \return A string containing the domain name.
   */
  std::string getName();

  /// Get the predicates existing in the domain.
  /**
   * \return The vector containing the name of the predicates.
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

  /// Get the derived predicates existing in the domain.
  /**
   * \return The vector containing the derived predicates.
   */
  std::vector<plansys2::Predicate> getDerivedPredicates();

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

  /// Get the details of a regular action existing in the domain.
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

  /// Get the details of a durative action existing in the domain.
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

  /// Get the current cached domain, ready to be saved to file, or to initialize another domain.
  /**
   * \param[in] use_cache Use cache, if available.
   * \return A string containing the domain.
   */
  std::string getDomain(bool use_cache);
  std::string cached_domain_;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<plansys2_msgs::srv::GetDomain>::SharedPtr get_domain_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainName>::SharedPtr get_name_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainTypes>::SharedPtr get_types_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainConstants>::SharedPtr get_constants_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_functions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_derived_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>::SharedPtr
    get_derived_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_actions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_durative_actions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr get_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr get_function_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActionDetails>::SharedPtr get_action_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainDurativeActionDetails>::SharedPtr
    get_durative_action_details_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr domain_sub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_
