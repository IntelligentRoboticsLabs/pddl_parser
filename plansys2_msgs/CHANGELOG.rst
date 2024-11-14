^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.14 (2024-11-14)
-------------------

2.0.13 (2024-11-06)
-------------------

2.0.12 (2024-10-16)
-------------------
* add uint8 EXISTS = 10 to Node.msg
* support for ParamExpression and ConstExpression
* add Derived.msg and GetDomainDerivedPredicateDetails.srv
* Add option to use planner node to validate domain in domain expert
* Contributors: Francisco Martín Rico, Gustavo, Josh Zapf, Marco Roveri, Robodrome, Sebastian Castro

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

2.0.4 (2022-05-03)
------------------
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Jake Keller

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------
* Full supportr for (= _ _) PDDL constraints, added possibility to refer to constants, added initial test cases
* Add Status recency in performers
* Pddl domain (:constants ) handling and planning
* [domain_expert] GetDomainConstants service and tests
* [msgs] GetDomainConstants srv
* Contributors: Francisco Martín Rico, Jake Keller, Josh Zapf, Marco Roveri, Mostafa Gomaa

2.0.1 (2022-02-03)
------------------
* get-domain-name: Added service for retrieving domain name. Concatenating domain names when using multiple domains.
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Josh Zapf

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* ros2-plan-msg: Adding functions to plansys2_msgs/Knowledge message.
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Merging master and resolving conflicts.
* Using custom behavior tree to enable action timeouts.
* pddl-tree-messages: Adding README to plansys2_msgs.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Plansys2_tests package
* Add GetOrderedSubGoals service to Executor, allowing executor clients to get the order in which sub-goals will be completed by the current plan.
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Action execution refactoring
* Adding isGoalSatisfied function and unit tests to the problem expert api.
* Fixing typo in message name.
* Add support for numeric conditions and effects.
* Remove unused field
* Publish knownledge content
* Monitorization info
* fix minor typo
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Josh Zapf

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
* Contributors: Francisco Martín Rico
0.0.8 (2020-07-18)
------------------

0.0.7 (2020-03-26)
------------------

0.0.6 (2020-03-23)
------------------

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
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Executor initial version
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert client and node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Domain types and messages changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of domain expert
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico