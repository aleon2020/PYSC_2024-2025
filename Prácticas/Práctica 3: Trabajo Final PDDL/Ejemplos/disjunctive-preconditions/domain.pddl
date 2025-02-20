(define (domain disjunctive-preconditions)
(:requirements :strips :typing :disjunctive-preconditions)

(:types
  location
  robot
  sample
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?s - sample)
  (high_robot ?r - robot)
  (low_robot ?r - robot)
  (high_location ?l - location)
  (low_location ?l - location)
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
      (gripper_free ?r)
      (or
        (and (high_robot ?r) (high_location ?l))
        (and (low_robot ?r) (low_location ?l))
      )
    )
  :effect
    (and
      (robot_carry ?r ?s)
      (not (sample_at ?s ?l))
      (not (gripper_free ?r))
    )
)

(:action drop
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (robot_carry ?r ?s)
      (high_robot ?r)
      (low_location ?l)
    )
  :effect 
    (and 
      (sample_at ?s ?l)
      (gripper_free ?r)
      (not (robot_carry ?r ?s))
    )
)

)