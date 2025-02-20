(define (problem universal-preconditions)
(:domain universal-preconditions)

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

(:goal
  (robot_at curiosity base)
)

)