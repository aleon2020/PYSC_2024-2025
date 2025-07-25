(define (domain exploration_example_domain)
(:requirements :strips :typing)

(:types
  location
  robot
  sample
  soil
  ; Nuevo tipo
  drill
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (soil_at ?s - soil ?l - location)
  (soil_analysed ?sl - soil)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?s - sample)
  ; Nuevos predicados
  ; robot_at: Indica la ubicación en la que se encuentra el taladro.
  (drill_at ?d - drill ?l - location)
  ; drill_free: Indica si el taladro está libre.
  (drill_free ?d - drill)
  ; robot_has_drill: Indica si el robot está llevando el taladro.
  (robot_has_drill ?r - robot ?d - drill) 
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:action move
  :parameters (?r - robot ?from ?to - location)
  :precondition
    (and 
      (robot_at ?r ?from)
    )
  :effect
    (and
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
)

; Collect action. The robot collects a sample at a location.
; Both the robot and the sample must be in that location.
; The robot's gripper must be free (the robot can only hold 1 sample).
; Consequences:
;   - The sample is no longer at the given location.
;   - The robot is now carrying the sampple and its gripper is not free.
(:action collect
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition 
    (and
      (sample_at ?s ?l)
      (robot_at ?r ?l)
      (gripper_free ?r)
    )
  :effect
    (and
      (robot_carry ?r ?s)
      (not (sample_at ?s ?l))
      (not (gripper_free ?r))
    )
)

; Drop-off action. The robot drops a sample at a location.
; The robot must be in that location and must be carrying that sample.
; Consequences:
;   - The sample is now at the given location.
;   - The robot is no longer carrying the sample and its gripper is free.
(:action drop
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (robot_carry ?r ?s)
    )
  :effect 
    (and 
      (sample_at ?s ?l)
      (gripper_free ?r)
      (not (robot_carry ?r ?s))
    )
)

; Analyse Soil action. The robot samples and analyses the soil at a location.
; The robot must be in that location.
; Consequences:
;   - The soil is now analysed.
(:action analyse-soil
  ; Nuevo parámetro ?d - drill
  :parameters (?s - soil ?l - location ?r - robot ?d - drill)
  :precondition
    (and 
      (robot_at ?r ?l)
      (soil_at ?s ?l)
      (gripper_free ?r)
      ; Nueva precondición
      ; El robot debe llevar el taladro para poder analizar el suelo.
      (robot_has_drill ?r ?d)
    )
  :effect
    (and
      (soil_analysed ?s)
    )
)

; Nueva acción collect-drill
; Collect Drill action. The robot picks up the drill at a location.
; The robot must be in that location, the drill must be there, and it must be free.
; Consequences:
;   - The robot now has the drill.
;   - The drill is no longer free.
;   - The drill is no longer at the given location.
(:action collect-drill
  :parameters (?r - robot ?d - drill ?l - location)
  :precondition
    (and
      (robot_at ?r ?l)
      (drill_at ?d ?l)
      (drill_free ?d)
    )
  :effect
    (and
      (robot_has_drill ?r ?d)
      (not (drill_free ?d))
      (not (drill_at ?d ?l))
    )
)

; Nueva acción drop-drill
; Drop Drill action. The robot drops the drill at a location.
; The robot must be in that location and must be carrying the drill.
; Consequences:
;   - The drill is now at the given location.
;   - The drill is free and available for use.
;   - The robot is no longer carrying the drill.
(:action drop-drill
  :parameters (?r - robot ?d - drill ?l - location)
  :precondition
    (and
      (robot_at ?r ?l)
      (robot_has_drill ?r ?d)
    )
  :effect
    (and
      (drill_at ?d ?l)
      (drill_free ?d)
      (not (robot_has_drill ?r ?d))
    )
)

)
