(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions :derived-predicates )

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
person
message
robot
room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_talk ?r - robot ?m - message ?p - person)
(robot_near_person ?r - robot ?p - person ?ro - room)
(robot_at ?r - robot ?ro - room)
(person_at ?p - person ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


(:derived (robot_near_person ?r ?p ?ro)
  (and (robot_at ?r ?ro) (person_at ?p ?ro))
)

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1)))
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action talk
    :parameters (?r - robot ?from ?p - person ?m - message ?ro - room)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_near_person ?r ?p ?ro))
    )
    :effect (and
        (at end(robot_talk ?r ?m ?p))
    )
)

;(:durative-action approach
;    :parameters (?r - robot ?ro - room ?p - person)
;    :duration ( = ?duration 5)
;    :condition (and
;        (over all(robot_at ?r ?ro))
;        (over all(person_at ?p ?ro))
;    )
;    :effect (and
;        (at end(robot_near_person ?r ?p))
;    )
;)

(:durative-action move_person
    :parameters (?p - person ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and 
        (at start(person_at ?p ?r1))
    )
    :effect (and
        (at start(person_at ?p ?r2))
        (at end(not(person_at ?p ?r1)))
    )
)



);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
