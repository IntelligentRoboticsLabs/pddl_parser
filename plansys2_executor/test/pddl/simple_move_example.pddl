(define (domain replan)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates


(robot_at ?r - robot ?wp - waypoint)
(connected ?wp_from ?wp_to - waypoint)


);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?from ?to - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?from))
        (over all(connected ?from ?to))
        )
    :effect (and
        (at end(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
