(define (problem simple_1)
  (:domain simple)
  (:objects
    leia - robot
    Jack - person
    kitchen bedroom - room
    m1 - message
  )
  (:init
    (robot_at leia kitchen)


  )

  ;; The goal is to have both packages delivered to their destinations:
  (:goal (and
    (robot_talk leia m1 Jack) 
    )
  )
  )
