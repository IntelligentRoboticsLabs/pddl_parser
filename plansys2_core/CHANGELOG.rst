^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.14 (2024-11-14)
-------------------

2.0.13 (2024-11-06)
-------------------

2.0.12 (2024-10-16)
-------------------
* Integrated feedback, and fixes to have tests to pass
* Added support for configuring the planner timeout
* Remove cmake warning
* Add option to use planner node to validate domain in domain expert
* Remove reference to SharedPtr
* Contributors: Francisco Martín Rico, Gustavo, Josh Zapf, Marco Roveri, Robodrome, Sebastian Castro, Splinter1984

2.0.9 (2022-07-10)
------------------

2.0.8 (2022-05-04)
------------------
* Add missing dependency
* Contributors: Francisco Martín Rico

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

2.0.2 (2022-04-03)
------------------
* Add Thread dependency
* Contributors: Francisco Martín Rico

2.0.1 (2022-02-03)
------------------
* Remove pthread dependency
* Merge galactic-devel
* Contributors: Francisco Martín Rico

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* Disable runtime/explicit cpplint errors
* pddl-tree-messages: Removing explicit specifiers from plansys2_core/Types.hpp that appear to be causing build errors.
* pddl-tree-messages: Deleting unused function declarations. Adding explicit specifier to single param constructors in plansys2_core/Types.hpp.
* Suggestion for `#118 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/118>`_
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Using helper functions in problem expert client to handle user interactions.
* pddl-tree-messages: Using alternate constructor to instantiate Instance in toInstance function.
* pddl-tree-messages: Updating getInstance and getInstances calls to use helper classes.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Update version
* Contributors: Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Ignore comments on block end search
* PDDL transparent merge
* Contributors: Fabrice Larribe, Francisco Martín Rico
1.0.7 (2021-01-04)
------------------

1.0.6 (2020-12-29)
------------------

1.0.5 (2020-12-28)
------------------
* Migration to c++17
* Contributors: Francisco Martín Rico

1.0.4 (2020-12-24)
------------------
* Add missing package dep
* Contributors: Francisco Martín Rico
1.0.3 (2020-12-23)
------------------

1.0.2 (2020-12-23)
------------------
* Plan solvers as plugins
* Contributors: Fabrice Larribe, Francisco Martin Rico, f269858

1.0.1 (2020-07-19)
------------------

0.0.7 (2020-03-26)
------------------

0.0.6 (2020-03-23)
------------------

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09 07:55)
------------------------

0.0.3 (2020-01-09 07:11)
------------------------

0.0.2 (2020-01-08)
------------------
