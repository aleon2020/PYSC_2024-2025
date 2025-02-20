(define (domain durative-actions)
(:requirements :strips :typing :durative-actions)

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
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 20)
  :condition
    (and 
      (at start (robot_at ?r ?from))
    )
  :effect
    (and
      (at end (robot_at ?r ?to))
      (at start (not (robot_at ?r ?from)))
    )
)

(:durative-action pick
  :parameters (?s - sample ?l - location ?r - robot)
  :duration (= ?duration 10)
  :condition 
    (and
      (at start (sample_at ?s ?l))
      (over all  (robot_at ?r ?l))
      (at start (gripper_free ?r))
    )
:effect
  (and
    (at end (robot_carry ?r ?s))
    (at start (not (sample_at ?s ?l)))
    (at start (not (gripper_free ?r))))
  )

(:durative-action drop
:parameters (?s - sample ?l - location ?r - robot)
:duration (= ?duration 5)
:condition
  (and 
    (over all (robot_at ?r ?l))
    (at start (robot_carry ?r ?s))
  )
:effect 
  (and 
    (at end (sample_at ?s ?l))
    (at end (gripper_free ?r))
    (at start (not (robot_carry ?r ?s)))
  )
)

)
