(define (problem exploration_example_problem)
(:domain exploration_example_domain)

; We define 3 different samples, 2 types of trash bins, one table and a robot
(:objects
  curiosity - robot
  base valley crater - location
  rock mineral - sample
  valley_soil crater_soil - soil
)

; Initially the robot is at the base
(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (sample_at rock crater)
  (sample_at mineral valley)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
)

; The goal is to make science!
(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
  )
)

)
