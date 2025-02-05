(define (problem rover-problem)
(:domain rover)

 ;; Instantiate the objects.
(:objects
  curiosity - robot
  wp1 wp2 wp3 wp4 wp_recharge - waypoint
)

(:init
  ; Define the initial state predicates.
  (robot_at curiosity wp1)
  (connected wp1 wp2)
  (connected wp2 wp1)
  (connected wp1 wp3)
  (connected wp3 wp1)
  (connected wp1 wp4)
  (connected wp4 wp1)
  (connected wp2 wp3)
  (connected wp3 wp2)
  (connected wp2 wp4)
  (connected wp4 wp2)
  (connected wp3 wp4)
  (connected wp4 wp3)
  (connected wp2 wp_recharge)
  (connected wp_recharge wp2)
)

(:goal 
  (and
    (visit wp1)
    (visit wp2)
    (visit wp3)
    (visit wp4)
  )
)

)