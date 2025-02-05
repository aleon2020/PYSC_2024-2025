(define (domain exploration_example_domain)
(:requirements :strips :typing)

(:types
  location
  robot
  sample
  soil
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (soil_at ?s - soil ?l - location)
  (soil_analysed ?sl - soil)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?s - sample)
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
  :parameters (?s - soil ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (soil_at ?s ?l)
      ; Nueva precondición
      ; La pinza del robot debe estar libre.
      (gripper_free ?r)
    )
  :effect 
    (soil_analysed ?s)
)

)
