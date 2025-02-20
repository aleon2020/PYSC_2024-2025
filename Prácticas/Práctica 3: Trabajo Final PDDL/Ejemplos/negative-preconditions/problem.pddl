(define (problem negative-preconditions)
(:domain negative-preconditions)

(:objects
  curiosity - robot
  crater - location 
  rock mineral - sample
)

(:init
  (robot_at curiosity crater)
  (sample_at rock crater)
  (sample_at mineral crater)
)

(:goal (and
  (robot_at curiosity base)
  (robot_carry curiosity rock))
)

)