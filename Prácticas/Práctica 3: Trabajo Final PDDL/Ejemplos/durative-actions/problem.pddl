(define (problem durative-actions)
(:domain durative-actions)

(:objects
  curiosity - robot
  base crater valley volcano - location 
  rock mineral magma - sample
)

(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (sample_at rock crater)
  (sample_at mineral crater)
  (sample_at magma crater)
)

(:goal (and
    (sample_at rock base)
    (sample_at magma valley)
    (sample_at mineral volcano)
  )
)

)
