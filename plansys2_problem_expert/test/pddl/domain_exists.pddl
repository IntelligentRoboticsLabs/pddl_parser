(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions :existential-preconditions)
(:types
    robot
    room
)

(:predicates
    (robot_at ?r - robot ?ro - room)
    (connected ?ro1 ?ro2 - room)
    (battery_full ?r - robot)
    (charging_point_at ?ro - room)
)

(:constants
 rob1 rob2 - robot
 bathroom - room
)

(:functions
    (battery_level ?r - robot)
)

(:action action_test
    :parameters (?r - robot)
    :precondition (and
      (exists (?ro)
        (and
          (robot_at ?r ?ro)
          (charging_point_at ?ro)
        )
      )
      (and (> (battery_level ?r) 1) (< (battery_level ?r) 200))
    )
    :effect (and
      (battery_full ?r)
    )
)

(:action action_test2
    :parameters (?r - robot)
    :precondition
      (exists (?ro1 ?ro2)
        (and
          (robot_at ?r ?ro1)
          (connected ?ro1 ?ro2)
        )
      )
    :effect (and )
)


)
