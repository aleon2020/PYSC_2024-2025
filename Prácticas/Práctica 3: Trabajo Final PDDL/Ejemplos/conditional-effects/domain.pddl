(define (domain conditional-effects)
(:requirements :strips :typing :conditional-effects :equality)

(:types
  location
  robot
  sample
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (robot_carry ?r - robot ?s - sample)
  (robot_in_base ?r - robot)
)

(:constants
  base - location
  rock - sample
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
  :parameters (?r - robot ?from - location ?s - sample)
  :precondition
    (and 
      (robot_at ?r ?from)
    )
  :effect
    (and
      (robot_at ?r base)
      (not (robot_at ?r ?from))
      (when 
        (and (robot_carry ?r rock) (and (robot_carry ?r ?s) (not (= ?s rock)))) 
        (robot_in_base ?r)
      )
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
