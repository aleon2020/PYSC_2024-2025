(define (problem conditional-effects)
(:domain conditional-effects)

(:objects
  curiosity - robot
  crater - location
  mineral magma - sample
)

(:init
  (robot_at curiosity crater)
  (sample_at mineral crater)
  (sample_at magma crater)
  (sample_at rock crater)
)

(:goal
  (robot_in_base curiosity)
)

)
