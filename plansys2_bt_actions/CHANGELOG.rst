^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_bt_actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.13 (2024-11-06)
-------------------
* Change to EventsExecutor
* Contributors: Francisco Martín Rico

2.0.12 (2024-10-16)
-------------------
* Remove cmake warning
* Removed some small references still related to the old version of BehaviorTree
* Bump Behaviortree.CPP v3 to v4
* fix code style issues with uncrustify
* Update BTAction.cpp
* fix groot enabling
* Linting
* Fix logger destruction at BTActions
* Update Changelog
* Insert in blackboard the action ROS 2 Node
* Insert in blackboard the action ROS 2 Node
* Correct tick function call in bt_actions test
* Merge remote-tracking branch 'origin/master' into fix_goal_structure_issue_205
* Explicitly ignore feedback in default on_feedback
* Add try/catch to bt creation, reset loggers
* Contributors: Andrianov Roman, Francisco Martín Rico, Jake Keller, Marco Roveri, Splinter1984

2.0.9 (2022-07-10)
------------------
* Add BT logging, Tree execution try/catch, and README
* Add try/catch to BT execution in BTAction
* log INFO in BtActionNode when sending action goal
* Add BT logging to BTAction
* Contributors: Francisco Martín Rico, Jake Keller

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
* Update unit tests to match changes
* Update node name remapping to new syntax
* Update node name remapping to new syntax
* Merge branch 'origin/unique-node-names'
* Remap BB node name in source using NodeOptions
* Remove empty line to make cpplint happy
* Remove swap file
* Handle on_tick() during the first tick
* Remove holdover code from pre-refactor
  We should not set running explicity here as the user defined on_tick()
  would have no way to know that it is the first tick.
  The status will be set implicity by the parent class's execute_tick()
  method.
  Because the on_tick() method has been moved, a goal is no longer being
  set in the first block of code, making the first call to
  on_new_goal_received() errant.
* Remove on_wait_for_result
  The function is redundant now that on_tick() is called every tick
* Contributors: Francisco Martín Rico, Jake Keller

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------
* action-graph-fix: Reverting timeout modification for bt_action_test.
* Refactor BTActionNode (`#197 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/197>`_)
* action-test-fix: Setting TIMEOUT value to 300 for bt_action_test.
  Increase default server tiemout
* Replace runtime errors w/ BT::NodeStatus::FAILURE
* Add on_feedback method
* Fix issue if child classes override providedPorts
* Add (optional) server timeout to BtActionNode
* Contributors: Francisco Martín Rico, Jake Keller, Josh Zapf

2.0.1 (2022-02-03)
------------------
* Update deprecated APIs in launchers and parameters
* Fix plansys2_bt_actions CMakeLists.txt
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Jake Keller

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------
* Fix tests
* Fix rate conversions
* Reduce debug output
* Update version
* Contributors: Francisco Martín Rico

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Moving zmq publisher creation to on_activate in BTAction, resetting the publisher in on_deactivate.
* Action execution refactoring
* Adding zeromq-based groot monitoring of plansys2 behaviortree actions. Ports are not specified by default to keep two different actions from accidentally using the same ports.
* Add support for numeric conditions and effects.
* Monitorization info
* Improving BTActions
* Change 'move' action name
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Josh Zapf

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
