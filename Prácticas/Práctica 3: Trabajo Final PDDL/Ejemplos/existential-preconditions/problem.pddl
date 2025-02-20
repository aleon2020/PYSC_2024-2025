(define (problem existential-preconditions)
(:domain existential-preconditions)

(:objects
  curiosity - robot
  valley - location
  rock mineral - sample
)

(:init
  (robot_at curiosity valley)
  (sample_at rock valley)
  (sample_at mineral valley)
)

(:goal
  (robot_at curiosity base)
)

)
