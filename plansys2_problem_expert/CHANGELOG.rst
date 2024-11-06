^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_problem_expert
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.13 (2024-11-06)
-------------------
* Change to EventsExecutor
* Contributors: Francisco Martín Rico

2.0.12 (2024-10-16)
-------------------
* evaluate exists use_state==false
* evaluate exsits w/ problem_client
* add replace_children_param
* return derived predicates in getPredicates
* add cart_produdct and replace_children_param
* fix bug in ParamExpression getTree
* support for ParamExpression and ConstExpression
* add test for negated comp expression
* negate evaluation of comp expression
* add test problem_expert.exist_precidate
* in existPredicate check if derived predicate is satisfied
* Fixed support for constants in predicates/functions/goals
* Make problemExpert case-insensitive to Instance types
* Use separate problem pddl for problem expert.
* Minor changes to the tests to comply with some change in the generated output
* Fixed some errors and some tests to comply with the revised output
* Fixed support for the complex goal parsing both from the terminal and from file
* Minor changes to the tests to comply with some change in the generated output
* Check instance types and associated tests
* Fixed support for the complex goal parsing both from the terminal and from file
* Contributors: Ashton Larkin, Francisco Martín Rico, Gustavo, Josh Zapf, Marco Roveri, Mostafa Gomaa, Splinter1984, adfea625

2.0.9 (2022-07-10)
------------------
* Humble Upgrade
* Fix possible bug https://github.com/ros2/rclcpp/issues/1968
* Contributors: Francisco Martín Rico

2.0.8 (2022-05-04)
------------------

2.0.7 (2022-05-04)
------------------

2.0.6 (2022-05-03)
------------------

2.0.5 (2022-05-03)
------------------
* Fix ROS2 Buildfarm error due to Threads
* Contributors: Francisco Martín Rico, Jake Keller

2.0.4 (2022-05-03)
------------------
* Fix version
* Fix ROS2 Buildfarm error due to Threads
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Francisco Martín Rico, Jake Keller

2.0.3 (2022-04-03)
------------------
* Fix conflicts
* Qt5 dependencies
* Contributors: Francisco Martín Rico

2.0.2 (2022-04-03)
------------------
* Full supportr for (= _ _) PDDL constraints, added possibility to refer to constants, added initial test cases
* set_decimal_precision: Use std::setprecision to avoid incompatible scientific notation.
* [problem_expert] clearGoal on clearing knowledge
* [problem_expert] test constants parsing as part of problem
* [problem_expert] add constants as instances while parsing problem.pddl
* Contributors: Francisco Martín Rico, Jake Keller, Josh Zapf, Marco Roveri, Mostafa Gomaa

2.0.1 (2022-02-03)
------------------
* Update deprecated APIs in launchers and parameters
* get-domain-name: Added service for retrieving domain name. Concatenating domain names when using multiple domains.
* Update launcher param names
* remove-invalid-goals: Removing invalid goals when instances are removed. Keep track of update time in problem expert.
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Josh Zapf

2.0.0 (2021-07-04)
------------------
* Compile for ROS2 Galactic
* Contributors: Francisco Martín Rico, Jonatan Olofsson

1.0.10 (2021-07-03)
-------------------
* Remove node parameter in client constructors
* Adding comments specifying how the syntax is wrong in unexpected_syntax problem file.
* Adding unit tests for new functions in plansys2_problem_expert.
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* ros2-plan-msg: Adding functions to plansys2_msgs/Knowledge message.
* pddl-tree-messages: Deleting unused function declarations. Adding explicit specifier to single param constructors in plansys2_core/Types.hpp.
* Suggestion for `#118 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/118>`_
* pddl-tree-messages: Extending action string parsing utilities in plansys2_problem_expert/Utils.cpp.
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Replacing user access function calls with shorter versions where possible.
* pddl-tree-messages: Adding user access functions to domain expert client.
* pddl-tree-messages: Adding goal user access functions to problem expert client.
* pddl-tree-messages: Adding predicate user access functions to problem expert client.
* pddl-tree-messages: Adding predicate user access functions to problem expert client.
* pddl-tree-messages: Using helper functions in problem expert client to handle user interactions.
* pddl-tree-messages: Updating getInstance and getInstances calls to use helper classes.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Replacing hard coded node equality check with function call.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Adding support for PDDL addition and subtraction expressions. Courtesy of @jjzapf
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Add unit test checking that an empty goal is not valid.
* Add check for empty pointer prior to accessing pointer. Applies to checking the tree types of a predicate, involved in checking if a goal is valid.
* Fix typo in comment string to stop LCOV exclusion block.
* Update API for FutureReturnCode
* Action execution refactoring
* Adding unit tests for the Utils file. Fixing bug in plansys2_pddl_parser when getting predicate or function name that has no params it used to include the closing parenthesis in the name, which was incorrect. Making truth value false for expression arithmetic variants. fixing bug: not applying function modifier if trying to divide by zero.
* Adding isGoalSatisfied function and unit tests to the problem expert api.
* Moving Utils file to plansys2_problem_expert since all the functions are checking information in the problem or modifying the problem.
* Fixing typo in message name.
* Add support for numeric conditions and effects.
* Publish knownledge content
* PDDL transparent merge
* Fix the call of the get_problem_instance service
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Josh Zapf, Thomas Denewiler

1.0.7 (2021-01-04)
------------------
* Making explicit dependencies
* Contributors: Francisco Martín Rico
1.0.6 (2020-12-29)
------------------

1.0.5 (2020-12-28)
------------------
* Migration to c++17
* Contributors: Francisco Martín Rico

1.0.4 (2020-12-24)
------------------

1.0.3 (2020-12-23)
------------------

1.0.2 (2020-12-23)
------------------
* Plan solvers as plugins
* Contributors: Fabrice Larribe, Francisco Martin Rico, f269858

1.0.1 (2020-07-19)
------------------

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Contributors: Francisco Martin Rico


0.0.8 (2020-07-18)
------------------
* Boost:optional
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Run in separate namespaces. Monolothic node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Add multi domain
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09)
------------------
* Adding missing action dependencies
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.3 (2020-01-09)
------------------

0.0.2 (2020-01-08)
------------------
* Merge pull request `#16 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/16>`_ from IntelligentRoboticsLabs/pddl_parser_rename
  Rename pddl_parser
* Rename pddl_parser
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Merge pull request `#8 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/8>`_ from IntelligentRoboticsLabs/patrol_example
  Patrol example
* Patrol example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Adding documentation
  Signed-off-by: Francisco Martin Rico <fmartin@gsyc.urjc.es>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Execute actions independiently. Example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Update notification in problem
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert complete with terminal support
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert client and node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Goals in problem generation
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* ProblemExpert local complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Using shred_ptr. First commit Problem
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
