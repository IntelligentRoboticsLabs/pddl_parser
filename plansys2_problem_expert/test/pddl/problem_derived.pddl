(define (problem simple_1)
  (:domain simple)
  (:objects
    leia r2d2 turtlebot mirte tiago walle pluto donald rob1 eva rob2 - robot
    person1 person2 person3 person4 person5 boringperson boringperson2 boringperson3 boringperson4 boringperson5 Jack Alice bob jhon jose carlos marta dilma edson einstein albert lucas gustavo person6 person7 person8 person9 person10 - person
    kitchen bedroom livingroom garden office pool garage - room
  )
  (:init
    (robot_at leia kitchen)
    (robot_at r2d2 bedroom)
    (robot_at turtlebot livingroom)
    (robot_at mirte garden)
    (robot_at tiago office)
    (robot_at walle pool)
    (robot_at pluto garage)
    (robot_at donald kitchen)
    (robot_at rob1 bedroom)
    (robot_at eva kitchen)
    (robot_at rob2 livingroom)
    (robot_at drone123 drone_area)
    (person_at Jack bedroom)
    (person_at Alice bedroom)
    (person_at bob livingroom)
    (person_at Jhon livingroom)
    (person_at jose livingroom)
    (person_at carlos livingroom)
    (person_at marta livingroom)
    (person_at dilma livingroom)
    (person_at edson livingroom)
    (person_at einstein livingroom)
    (person_at albert livingroom)
    (person_at lucas livingroom)
    (person_at gustavo livingroom)
    (is_drone rob1)
    (is_drone drone123)
    (is_rocket rob2)

    (= (battery_level r2d2) 99)
    (= (battery_level rob1) 50)
  )

  ;; The goal is to have both packages delivered to their destinations:
  (:goal (and
      (party gustavo lucas albert einstein edson turtlebot rob2 livingroom)
    )
  )
)
