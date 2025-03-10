(define (domain exploration_example_domain)
(:requirements :strips :typing :durative-actions)


; Types definition
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
  (occupied_robot ?r - robot)
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 10)
  :condition
    (and 
      (at start (robot_at ?r ?from))
      (at start (occupied_robot ?r))
    )
  :effect
    (and
      (at start (not (occupied_robot ?r)))
      (at end (robot_at ?r ?to))
      (at start (not (robot_at ?r ?from)))
      (at end (occupied_robot ?r))
    )
)

; Collect action. The robot collects a sample at a location.
; Both the robot and the sample must be in that location.
; The robot's gripper must be free (the robot can only hold 1 sample).
; Consequences:
;   - The sample is no longer at the given location.
;   - The robot is now carrying the sampple and its gripper is not free.
(:durative-action collect
  :parameters (?s - sample ?l - location ?r - robot)
  :duration (= ?duration 4)
  :condition 
    (and
      (at start (sample_at ?s ?l))
      (at start (robot_at ?r ?l))
      (at start (gripper_free ?r))
      (at start (occupied_robot ?r))
    )
  :effect
    (and
      (at start (not (occupied_robot ?r)))
      (at end (robot_carry ?r ?s))
      (at start (not (sample_at ?s ?l)))
      (at start (not (gripper_free ?r)))
      (at end (occupied_robot ?r))
    )
)

; Drop-off action. The robot drops a sample at a location.
; The robot must be in that location and must be carrying that sample.
; Consequences:
;   - The sample is now at the given location.
;   - The robot is no longer carrying the sample and its gripper is free.
(:durative-action drop
  :parameters (?s - sample ?l - location ?r - robot)
  :duration (= ?duration 1)
  :condition
    (and 
      (at start (robot_at ?r ?l))
      (at start (robot_carry ?r ?s))
      (at start (occupied_robot ?r))
    )
  :effect 
    (and 
      (at start (not (occupied_robot ?r)))
      (at end (sample_at ?s ?l))
      (at end (gripper_free ?r))
      (at start (not (robot_carry ?r ?s)))
      (at end (occupied_robot ?r))
    )
)

; Analyse Soil action. The robot samples and analyses the soil at a location.
; The robot must be in that location.
; Consequences:
;   - The soil is now analysed.
(:durative-action analyse_soil
  :parameters (?s - soil ?l - location ?r - robot)
  :duration (= ?duration 15)
  :condition
    (and 
      (at start (robot_at ?r ?l))
      (at start (soil_at ?s ?l))
      (at start (occupied_robot ?r))
    )
  :effect 
    (and
      (at start (not (occupied_robot ?r)))
      (at end (soil_analysed ?s))
      (at end (occupied_robot ?r))
    )
)

)
