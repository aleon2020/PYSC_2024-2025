(define (domain negative-preconditions)
(:requirements :strips :typing :negative-preconditions)

(:types
  location
  robot
  sample
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (robot_carry ?r - robot ?s - sample)
)

(:constants
  base - location
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

(:action pick
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition 
    (and 
      (sample_at ?s ?l)
      (robot_at ?r ?l)
      (not (robot_carry ?r ?s))
    )
  :effect
    (and
      (robot_carry ?r ?s)
      (not (sample_at ?s ?l))
    )
)

)