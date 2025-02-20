(define (problem disjunctive-preconditions)
(:domain disjunctive-preconditions)

(:objects
  curiosity opportunity - robot
  base crater - location
  rock - sample
)

(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (robot_at opportunity crater)
  (gripper_free opportunity)
  (high_robot curiosity)
  (low_robot opportunity)
  (high_location base)
  (low_location crater)
  (sample_at rock base)
)

(:goal
  (robot_carry opportunity rock)
)

)
