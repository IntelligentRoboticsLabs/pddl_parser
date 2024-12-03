^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_domain_expert
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.15 (2024-12-03)
-------------------

2.0.14 (2024-11-14)
-------------------

2.0.13 (2024-11-06)
-------------------
* Change to EventsExecutor
* Contributors: Francisco Martín Rico

2.0.12 (2024-10-16)
-------------------
* include "object" in getType
* fix code style
* add params to getDerivedPredicate
* add tests for getDerivedPredicate(s)
* add getDerivedPredicate
* add getDerivedPredicates
* add DomainReader::get_derived_predicates
* Small optimization
* Fixed expected output
* Remove cmake warning
* Fixed dump of functions/constants/types to handle the case where no funcions/constants/types declared to avoid printing the entry, thus generating a valid PDDL file
* Remove commented-out line in domain expert node
* Switch to MultiThreadedExecutor
* Spin up temporary node for domain validation
* Add option to use planner node to validate domain in domain expert
* Reduce copypasta in domain expert
* Configure POPF solver in domain expert
* Merge remote-tracking branch 'upstream/master' into return-stn
* Merge remote-tracking branch 'origin/humble-devel'
* Reverting changes to checking for valid domain.
* Supporting time-triggered execution.
* Contributors: Francisco Martín Rico, Gustavo, Josh Zapf, Marco Roveri, Robodrome, Sebastian Castro, Splinter1984

2.0.9 (2022-07-10)
------------------
* Humble Upgrade
* Fix possible bug https://github.com/ros2/rclcpp/issues/1968
* Update popf checker interface
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
* Contributors: Francisco Martín Rico

2.0.4 (2022-05-03)
------------------
* Fix version
* Fix ROS2 Buildfarm error due to Threads
* Contributors: Francisco Martín Rico

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------
* Merge pull request `#139 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/139>`_ from MostafaGomaa/constants_params_handling
* [problem_expert] clearGoal on clearing knowledge
* [domain_expert] GetDomainConstants service and tests
* [domain_expert] test domain_reader constants
* [domain_expert] adapt tests, add constants to existing domain_processed
* [domain_expert] DomainReader, Extract "(:constants )" string from domain_str
* Contributors: Francisco Martín Rico, Jake Keller, Mostafa Gomaa

2.0.1 (2022-02-03)
------------------
* Update deprecated APIs in launchers and parameters
* get-domain-name: Adding unit test for GetDomainName service.
* get-domain-name: Added service for retrieving domain name. Concatenating domain names when using multiple domains.
* Update launcher param names
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Josh Zapf

2.0.0 (2021-07-04)
------------------
* Compile for ROS2 Galactic
* Contributors: Francisco Martín Rico, Jonatan Olofsson

1.0.10 (2021-07-03)
-------------------
* Remove node parameter in client constructors
* Adding unit tests for new functions in plansys2_domain_expert.
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* Suggestion for `#118 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/118>`_
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Adding user access functions to domain expert client.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Update API for FutureReturnCode
* Add support for numeric conditions and effects.
* New Graph creation Algorithm
* Added negative predicates support
* Ignore comments on block end search
* Add more verbose output to pddl errors
* PDDL transparent merge
* Contributors: Fabrice Larribe, Francisco Martin Rico, Josh Zapf

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

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Boost:optional
* Support for BT actions
* Contributors: Francisco Martin Rico

0.0.8 (2020-07-18)
------------------
* Support for BT actions
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
* Avoid inserting duplicate types
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Avoid inserting duplicate types
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
* Adding documentation
  Signed-off-by: Francisco Martin Rico <fmartin@gsyc.urjc.es>
* Adding documentation
  Signed-off-by: Francisco Martin Rico <fmartin@gsyc.urjc.es>
* Merge pull request `#6 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/6>`_ from IntelligentRoboticsLabs/documentation
  Documentation for Domain Expert
* Documentation for Domain Expert
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
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
* Domain types and messages changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Planning terminal and domain clients
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of domain expert
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
