(define (domain simple)
(:requirements :strips :typing :adl :derived-predicates :fluents)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
  person - object
  robot - object
  room - object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

(:constants drone123 - robot drone_area - room)

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
  (party ?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
  (robot_at ?r - robot ?ro - room)
  (person_at ?p - person ?ro - room)
  (is_drone ?r - robot)
  (is_rocket ?r)
  (inferred-robot_at ?r - robot ?ro - room)
  (inferred-exists-another-drone-at-drone_area ?r - robot)
  (inferred-person_at ?p - person ?ro - room)
  (inferred-party ?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
  (inferred-exists-party-in-room ?room)
  (inferred-aerial ?r - robot)
  (inferred-not_aerial ?r - robot)
  (inferred-drone_has_battery_level ?r - robot)
  (inferred-drone123 ?r - robot)
  (inferred-not-drone123 ?r - robot)
  (inferred-same-drone ?r ?r2 - robot)
  (inferred-not-same-drone ?r ?r2 - robot)
  (inferred-exists-equal-drone ?r)
  (inferred-exists-another-drone ?r)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;

(:functions
  (battery_level ?r - robot)
)

;; Derived predicates ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:derived (inferred-robot_at ?r - robot ?ro - room)
  (and
    (robot_at ?r ?ro)
  )
)
(:derived (inferred-exists-another-drone-at-drone_area ?r - robot)
  (and
    (is_drone ?r)
    (exists (?r)
      (and
        (robot_at ?r drone_area)
        (not (= ?r drone123))
      )
    )
  )
)

(:derived (inferred-person_at ?p - person ?ro - room)
  (and
    (person_at ?p ?ro)
  )
)

(:derived (inferred-aerial ?r - robot)
  (or
    (is_drone ?r)
    (is_rocket ?r)
  )
)

(:derived (inferred-not_aerial ?r - robot)
  (and
    (not (is_drone ?r))
    (not (is_rocket ?r))
  )
)

(:derived (inferred-party ?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
  (and
    (inferred-person_at ?p ?room)
    (inferred-person_at ?p2 ?room)
    (inferred-person_at ?p3 ?room)
    (inferred-person_at ?p4 ?room)
    (inferred-person_at ?p5 ?room)
    (inferred-robot_at ?r ?room)
    (inferred-robot_at ?r2 ?room)
  )
)

(:derived (inferred-exists-party-in-room ?room)
  (and
    (exists (?p ?p2 ?p3 ?p4 ?p5 ?r ?r2)
      (and
        (inferred-party ?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
      )
    )
  )
)

(:derived (inferred-drone_has_battery_level ?r - robot)
  (and
    (= (battery_level ?r) 99)
    (is_drone ?r)
  )
)

(:derived (inferred-drone123 ?r - robot)
  (and
    (= ?r drone123)
    (is_drone ?r)
  )
)

(:derived (inferred-not-drone123 ?r - robot)
  (and
    (not (= ?r drone123))
    (is_drone ?r)
  )
)

(:derived (inferred-same-drone ?r ?r2 - robot)
  (and
    (is_drone ?r)
    (is_drone ?r2)
    (= ?r ?r2)
  )
)

(:derived (inferred-not-same-drone ?r ?r2 - robot)
  (and
    (is_drone ?r)
    (is_drone ?r2)
    (not (= ?r ?r2))
  )
)

(:derived (inferred-exists-equal-drone ?r)
  (and
    (exists (?r2)
      (and
        (inferred-same-drone ?r ?r2)
      )
    )
  )
)

(:derived (inferred-exists-another-drone ?r)
  (and
    (exists (?r2)
      (and
        (is_drone ?r)
        (is_drone ?r2)
        (not (= ?r ?r2))
      )
    )
  )
)

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:action party
  :parameters (?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
  :precondition (and
    (inferred-party ?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
  )
  :effect (and
    (party ?p ?p2 ?p3 ?p4 ?p5 ?r ?r2 ?room)
  )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
