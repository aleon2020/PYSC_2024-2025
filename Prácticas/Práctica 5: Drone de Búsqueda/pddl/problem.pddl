(define (problem search_problem)
(:domain search_domain)

(:objects
  z1 z2 z3 z4 - location
  base - location
  drone_1 - drone
  pers_1 pers_2 pers_3 pers_4 - person
)

(:init
  ; Robot initialization
  (drone_at drone_1 base)

  ; Connections
  (connected base z1)
  (connected z1 base)
  (connected base z4)
  (connected z4 base)
  (connected z1 z2)
  (connected z2 z1)
  (connected z1 z3)
  (connected z3 z1)

  ; Person locations
  (person_at pers_1 z1)
  (person_at pers_2 z2)
  (person_at pers_3 z3)
  (person_at pers_4 z4)

)

; The goal is to rescue everyone
(:goal
  (and
    (person_found pers_1)
    (person_found pers_2)
    (person_found pers_3)
    (person_found pers_4)
  )
)

)
