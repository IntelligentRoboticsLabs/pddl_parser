^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.13 (2024-11-06)
-------------------
* Change to EventsExecutor
* Contributors: Francisco Martín Rico

2.0.12 (2024-10-16)
-------------------
* add get model derived predicates to terminal
* Added some information in the README to allow for using a configuration file to change the planner timeout
* Read multiple lines in terminal input
* Fix Terminal and executor bugs
* Change MultiThreaded for SingleThreaded in CI failing tests
* Fix terminal tests
* Contributors: Andrianov Roman, Francisco Martín Rico, Gustavo, Josh Zapf, Marco Roveri, Splinter1984

* Merge remote-tracking branch 'origin/humble-devel'
* Merge pull request `#251 <https://github.com/PlanSys2/ros2_planning_system/issues/251>`_ from PlanSys2/fix_bt_node
  Fix bt node
* Change MultiThreaded for SingleThreaded in CI failing tests
* Merge pull request `#238 <https://github.com/PlanSys2/ros2_planning_system/issues/238>`_ from roveri-marco/fix_goal_structure_issue_205
  Fix goal structure issue 205
* Fix terminal tests
* Minor fix
* Merge remote-tracking branch 'origin/master' into fix_goal_structure_issue_205
* Merge remote-tracking branch 'upstream/master'
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Andrianov Roman, Francisco Martín Rico, Marco Roveri, Splinter1984

2.0.9 (2022-07-10)
------------------

2.0.8 (2022-05-04)
------------------

2.0.7 (2022-05-04)
------------------

2.0.6 (2022-05-03)
------------------

2.0.5 (2022-05-03)
------------------
* Fix ROS2 Buildfarm error due to Threads
* Contributors: Francisco Martín Rico, Jake Keller, Marco Roveri

2.0.4 (2022-05-03)
------------------
* Fix version
* Fix ROS2 Buildfarm error due to Threads
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Merge pull request `#213 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/213>`_ from roveri-marco/fix_issue_predicate_no_args_in_terminal
  Fix for issue `#212 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/212>`_ - predicate with no args
* Fix for issue `#212 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/212>`_ - predicate with no args
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Francisco Martín Rico, Jake Keller, Marco Roveri

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------
* Made latest modifications reentrant without the use of global variables
* Renamed planfrom to plan-file, added help for help
* Added tests for source and run planfrom, cleaned code to have all the test to pass
* Fixed recursive load, added command run planfrom to read a plan from file and execute it
* Initial preliminary support for help and load new commands
* Fix run entire plan
* Add options to run command
* Add action failure details to ExecutorClient
* Contributors: Francisco Martín Rico, Jake Keller, Josh Zapf, Marco Roveri

2.0.1 (2022-02-03)
------------------
* Translate error message to English
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Ricardo Marques

2.0.0 (2021-07-04)
------------------
* Fix compile issues for galactic
* Compile for ROS2 Galactic
* Contributors: Francisco Martín Rico, Jonatan Olofsson, bjnjo

1.0.10 (2021-07-03)
-------------------
* Revert wrong directory move
* Remove node parameter in client constructors
* Enabling code coverage of plansys2_terminal::Terminal node init functions.
* Adding unit tests for new functions in plansys2_terminal.
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* Linting
* Fix set predicates with no arguments
* Suggestion for `#118 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/118>`_
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Replacing user access function calls with shorter versions where possible.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Merging upstream master and resolving conflicts.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Fix tests and linting
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Action execution refactoring
* Add support for numeric conditions and effects.
* Monitorization info
* fix minor typo
* Adding actor checker in terminal
* Remove debugging traces
* Fix the call of the get_problem_instance service
* Contributors: Fabrice Larribe, Francisco Martin Rico, Josh Zapf

1.0.7 (2021-01-04)
------------------
* Making explicit dependencies
* Contributors: Francisco Martín Rico
1.0.6 (2020-12-29)
------------------
* Disable boost functions
* Contributors: Francisco Martín Rico
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
* Boost:optional
* Contributors: Francisco Martin Rico


0.0.8 (2020-07-18)
------------------
* Add BT support
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Fix spaces in command line
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Terminal completion functionality
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
* Add readline dependency
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.2 (2020-01-08)
------------------
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Improved stdin read
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First functional version complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Execute actions independiently. Example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Executor initial version
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Update notification in problem
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert complete with terminal support
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert client and node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Domain types and messages changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Planning terminal and domain clients
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico