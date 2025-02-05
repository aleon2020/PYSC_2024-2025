(define (problem deposit_24)
(:domain deposit_fluent)

; We define 3 different items, 2 types of trash bins, one table and a robot
(:objects
  table floor - location
  bottle newspaper rotten_apple - item
  walle - robot
)

; Initially everything is on the floor and robot is by the table
(:init

  ; Item's weights
  (= (item_weight bottle) 1)
  (= (item_weight newspaper) 1)
  (= (item_weight rotten_apple) 1)
  
  ; Deposit's capacity
  (= (maximum_deposit_weight walle) 5)
  (= (actual_deposit_weight walle) 0)

  (robot_at walle table)
  (gripper_free walle)
  (item_at bottle floor)
  (item_at newspaper floor)
  (item_at rotten_apple floor)
)

; The goal is to clean the floor!
(:goal (and
    (item_in_deposit bottle)
    (item_in_deposit rotten_apple)
    (item_in_deposit newspaper)
  )
)

)
