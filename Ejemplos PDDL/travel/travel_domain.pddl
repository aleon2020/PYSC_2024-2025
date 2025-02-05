(define (domain travel)

  (:requirements :typing :fluents :durative-actions)

  (:types vehicle city - object)

  (:predicates
    (vehicle_at ?v - vehicle ?c - city)
    (connected ?c1 ?c2 - city)
  )

  (:functions
    (speed ?c - vehicle)
    (distance_traveled ?c - vehicle)
    (distance ?c1 ?c2 - city)
  )

  (:action drive
    :parameters (?v - vehicle ?c1 ?c2 - city)
    :precondition (and (connected ?c1 ?c2)
                    (vehicle_at ?v ?c1)
                    )
    :effect (and (not (vehicle_at ?v ?c1))
                 (vehicle_at ?v ?c2)
                 (increase (distance_traveled ?v) (distance ?c1 ?c2))
                 )
  )  

)