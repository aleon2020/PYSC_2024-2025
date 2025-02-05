(define (problem exploration_example_problem)
(:domain exploration_example_domain)

(:objects
  ; rover: Nuevo objeto de tipo robot.
  curiosity rover - robot
  ; crater, pit, dune y volcano: Nuevos objetos de tipo location.
  base valley crater pit dune volcano - location
  ; crystal, water y lava: Nuevos objetos de tipo sample.
  rock mineral crystal water lava - sample
  valley_soil crater_soil - soil
)

(:init
  (robot_at curiosity base)
  ; robot_at rover base: El robot rover se encuentra ubicado en la base.
  (robot_at rover base)
  (gripper_free curiosity)
  ; gripper_free curiosity: El robot rover no lleva nada en su pinza.
  (gripper_free rover)
  (sample_at rock crater)
  (sample_at mineral valley)
  ; sample_at lava volcano: La lava se encuentra ubicada en el volcán.
  (sample_at lava volcano)
  ; sample_at crystal pit: El cristal se encuentra ubicado en la fosa.
  (sample_at crystal pit)
  ; sample_at water dune: El agua se encuentra ubicada en la duna.
  (sample_at water dune)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
)

(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    ; sample_at lava base: La lava debe terminar ubicada en la base.
    (sample_at lava base)
    ; sample_at crystal base: El cristal debe terminar ubicado en la base.
    (sample_at crystal base)
    ; sample_at water base: El agua debe terminar ubicada en la base.
    (sample_at water base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
  )
)

)