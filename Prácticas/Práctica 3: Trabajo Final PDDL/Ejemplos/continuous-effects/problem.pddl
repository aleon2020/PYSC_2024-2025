(define (problem continuous-effects)
(:domain continuous-effects)

(:objects
  valley crater - location
  curiosity - robot
)

(:init
  (= (distance valley crater) 30)
  (= (distance crater valley) 30)
  (= (distance crater base) 20)
  (= (distance base crater) 20)
  (= (distance valley base) 50)
  (= (distance base valley) 50)
  (robot_at curiosity valley)
  (= (battery_level curiosity) 200)
  (= (consumption curiosity) 20)
  (move_faster crater)
)

(:goal (and
    (robot_at curiosity base)
    (< (battery_level curiosity) 100)
  )
)

)