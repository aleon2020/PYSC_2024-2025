(define (problem numeric-fluents)
(:domain numeric-fluents)

(:objects
  curiosity - robot
  base crater - location
  rock mineral crystal magma - sample
)

(:init
  (= (sample_weight rock) 50)
  (= (sample_weight mineral) 10)
  (= (sample_weight crystal) 5)
  (= (sample_weight magma) 30)
  (= (battery_level curiosity) 30)
  (robot_at curiosity crater)
  (sample_at rock base)
  (sample_at mineral crater)
  (sample_at crystal crater)
  (sample_at magma base)
)

(:goal 
  (> (battery_level curiosity) 95)
)

)
