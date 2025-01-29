(define (problem exploration_example_problem)
(:domain exploration_example_domain)

; We define 3 different samples, 2 types of trash bins, one table and a robot
(:objects
  curiosity rover - robot
  base valley crater pit dune volcano - location
  rock mineral crystal water lava - sample
  valley_soil crater_soil - soil
)

; Initially the robot is at the base
(:init
  (robot_at curiosity base)
  (robot_at rover base)
  (gripper_free curiosity)
  (gripper_free rover)
  (sample_at rock crater)
  (sample_at mineral valley)
  (sample_at lava volcano)
  (sample_at crystal pit)
  (sample_at water dune)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
)

; The goal is to make science!
(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    (sample_at lava base)
    (sample_at crystal base)
    (sample_at water base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
  )
)

)
