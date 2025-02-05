(define (problem exploration_example_problem)
(:domain exploration_example_domain)

(:objects
  curiosity - robot
  base valley crater - location
  rock mineral - sample
  valley_soil crater_soil - soil
)

(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (sample_at rock crater)
  (sample_at mineral valley)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
)

(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
  )
)

)
