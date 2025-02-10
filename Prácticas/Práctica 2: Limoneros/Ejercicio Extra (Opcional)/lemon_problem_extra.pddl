(define (problem lemon_extra)
(:domain lemon_extra)

(:objects
  base limonero1 limonero2 limonero3 limonero4 limonero5 - location
  limon1 limon2 limon3 limon4 limon5 limon6 limon7 limon8 limon9 limon10 - lemon
  walle - robot
  collection_basket - basket
)

(:init
  (connected limonero1 limonero2)
  (connected limonero2 limonero1)
  (connected limonero2 base)
  (connected base limonero2)
  (connected limonero3 limonero4)
  (connected limonero3 base)
  (connected limonero4 limonero3)
  (connected limonero4 base)
  (connected limonero5 base)
  (connected base limonero3)
  (connected base limonero4)
  (connected base limonero5)

  (= (locations_distance limonero1 limonero2) 2.5)
  (= (locations_distance limonero2 limonero1) 2.5)
  (= (locations_distance limonero2 base) 7.5)
  (= (locations_distance base limonero2) 7.5)
  (= (locations_distance limonero3 limonero4) 2.5)
  (= (locations_distance limonero3 base) 7.5)
  (= (locations_distance limonero4 limonero3) 2.5)
  (= (locations_distance limonero4 base) 7.5)
  (= (locations_distance limonero5 base) 7.5)
  (= (locations_distance base limonero3) 7.5)
  (= (locations_distance base limonero4) 7.5)
  (= (locations_distance base limonero5) 7.5)

  (robot_at walle base)
  (basket_at collection_basket base)
  (gripper_free walle)

  (lemon_tree limonero1)
  (lemon_tree limonero2)
  (lemon_tree limonero3)
  (lemon_tree limonero4)
  (lemon_tree limonero5)

  (lemon_at limon1 limonero1)
  (lemon_at limon2 limonero1)
  (lemon_at limon3 limonero2)
  (lemon_at limon4 limonero2)
  (lemon_at limon5 limonero3)
  (lemon_at limon6 limonero3)
  (lemon_at limon7 limonero4)
  (lemon_at limon8 limonero4)
  (lemon_at limon9 limonero5)
  (lemon_at limon10 limonero5)

  (= (max_basket_capacity collection_basket) 50)
  (= (current_basket_weight collection_basket) 0)
  
  (= (lemon_weight limon1) 10)
  (= (lemon_weight limon2) 10)
  (= (lemon_weight limon3) 10)
  (= (lemon_weight limon4) 10)
  (= (lemon_weight limon5) 10)
  (= (lemon_weight limon6) 10)
  (= (lemon_weight limon7) 10)
  (= (lemon_weight limon8) 10)
  (= (lemon_weight limon9) 10)
  (= (lemon_weight limon10) 10)
)

(:goal
  (and 
    (lemon_at limon1 base)
    (lemon_at limon2 base)
    (lemon_at limon3 base)
    (lemon_at limon4 base)
    (lemon_at limon5 base)
    (lemon_at limon6 base)
    (lemon_at limon7 base)
    (lemon_at limon8 base)
    (lemon_at limon9 base)
    (lemon_at limon10 base)
  )
)

)