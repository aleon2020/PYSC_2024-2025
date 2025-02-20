(define (domain numeric-fluents)
(:requirements :strips :typing :numeric-fluents)

(:types
  location
  robot
  sample
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
)

(:functions
  (battery_level ?r - robot)
  (sample_weight ?s - sample)
)

(:action move
  :parameters (?r - robot ?from ?to - location)
  :precondition
    (and 
      (robot_at ?r ?from)
    )
  :effect
    (and
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
)

(:action transport
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition 
    (and
      (sample_at ?s ?l)
      (robot_at ?r ?l)
    )
:effect
  (and
    (increase (battery_level ?r) (sample_weight ?s))
    (not (sample_at ?s ?l))
  )
)

)
