(define (problem exploration_problem_drill)
(:domain exploration_domain_drill)

(:objects
  curiosity peepo - robot
  valley crater forest broken_spaceship ocean cave - interest_point
  base - base
  rock mineral plant scrap wood - sample
  valley_soil crater_soil forest_soil ocean_soil cave_soil - soil
  drill - tool
)

(:init
  (robot_at curiosity base)
  (robot_at peepo base)

  (gripper_free curiosity)
  (gripper_free peepo)

  (tool_at drill base)

  (sample_at rock crater)
  (sample_at plant valley)
  (sample_at wood forest)
  (sample_at scrap broken_spaceship)
  (sample_at mineral cave)

  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
  (soil_at forest_soil forest)
  (soil_at ocean_soil ocean)
  (soil_at cave_soil cave)

  (connected base valley)
  (connected valley base)
  (connected forest valley)
  (connected valley forest)
  (connected forest ocean)
  (connected ocean forest)
  (connected base ocean)
  (connected ocean base)
  (connected base cave)
  (connected cave base)
  (connected base crater)
  (connected crater base)
  (connected broken_spaceship crater)
  (connected crater broken_spaceship)


  (= (robot_max_load curiosity) 10)
  (= (robot_max_load peepo) 5)

  (= (sample_weight rock) 8)
  (= (sample_weight plant) 2)
  (= (sample_weight wood) 3)
  (= (sample_weight scrap) 7)
  (= (sample_weight mineral) 5)
)

(:goal
  (and
    (all_samples_at_base base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
    (soil_analysed forest_soil)
    (soil_analysed ocean_soil)
    (soil_analysed cave_soil)
  )
)

)