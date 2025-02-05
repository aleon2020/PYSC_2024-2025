(define (problem lemon_fluent)
(:domain lemon_fluent)

(:objects
  base limonero1 limonero2 limonero3 limonero4 limonero5 - location
  limon1 limon2 limon3 limon4 limon5 limon6 limon7 limon8 limon9 limon10 - lemon
  robot1 - robot
)

(:init
  ; Conexiones entre ubicaciones
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

  ; Ubicación inicial del robot y la cesta
  (robot_at robot1 base)
  (gripper_free robot1)

  ; Ubicación de los limones en los árboles
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
    
  ; Pesos y capacidades
  (= (max_basket_capacity robot1) 50)
  (= (current_basket_weight robot1) 0)
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