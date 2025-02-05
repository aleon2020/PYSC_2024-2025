(define (problem exploration_example_problem)
(:domain exploration_example_domain)

(:objects
  curiosity - robot
  base valley crater - location
  rock mineral - sample
  valley_soil crater_soil - soil
  ; drill_tool: Nuevo objeto de tipo drill.
  drill_tool - drill
)

(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (sample_at rock crater)
  (sample_at mineral valley)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
  ; drill_at drill_tool base: El taladro se encuentra ubicado en la base.
  (drill_at drill_tool base)
  ; drill_free drill_tool: El taladro no ha sido cogido por ningún robot.
  (drill_free drill_tool)
)

(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
    ; drill_at drill_tool base: El taladro debe terminar ubicado en la base.
    (drill_at drill_tool base)
  )
)

)
