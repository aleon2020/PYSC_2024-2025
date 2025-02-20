(define (domain universal-preconditions)
(:requirements :strips :typing :universal-preconditions :equality)

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
      (not (= ?to base))
    )
  :effect
    (and
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
)

(:action return-to-base
  :parameters (?r - robot ?from - location)
  :precondition
    (and 
      (robot_at ?r ?from)
      (forall (?s - sample)
          (robot_carry ?r ?s)
      )
    )
  :effect
    (and
      (robot_at ?r base)
      (not (robot_at ?r ?from))
    )
)

(:action pick
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition 
    (and 
      (sample_at ?s ?l)
      (robot_at ?r ?l)
    )
  :effect
    (and
      (robot_carry ?r ?s)
      (not (sample_at ?s ?l))
    )
)

)