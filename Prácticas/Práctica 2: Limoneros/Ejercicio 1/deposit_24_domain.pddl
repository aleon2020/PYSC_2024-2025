(define (domain deposit_24)
(:requirements :strips :typing :fluents)

; Types definition
(:types
  location
  robot
  item
)

; New location
(:constants
  Large-deposit - location
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (item_at ?it - item ?l - location)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?it - item)
  ; New predicates
  (store_item ?r - robot ?it - item)
  (item_in_deposit ?it - item)
)

; Functions
(:functions
  (item_weight ?it - item)
  (maximum_deposit_weight ?r - robot)
  (actual_deposit_weight ?r - robot)
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

; Pick-up action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
;     - The item is no longer at the given location.
;     - The robot is now carrying the object and its gripper is not free.
(:action pick
  :parameters (?it - item ?l - location ?r - robot)
  :precondition 
    (and
      (item_at ?it ?l)
      (robot_at ?r ?l)
      (gripper_free ?r)
    )
:effect
  (and
    (robot_carry ?r ?it)
    (not (item_at ?it ?l))
    (not (gripper_free ?r)))
  )

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
;     - The item is now at the given location.
;     - The robot is no longer carrying the object and its gripper is free.
(:action drop
:parameters (?it - item ?l - location ?r - robot)
:precondition
  (and 
    (robot_at ?r ?l)
    (robot_carry ?r ?it)
  )
:effect 
  (and 
    (item_at ?it ?l)
    (gripper_free ?r)
    (not (robot_carry ?r ?it))
  )
)

; Load action. The robot loads an object into its container. 
; The object must be grasped by the gripper.
(:action load
  :parameters (?it - item ?r - robot)
  :precondition 
    (and
      (> (maximum_deposit_weight ?r) (+ (actual_deposit_weight ?r) (item_weight ?it)))
      (robot_carry ?r ?it)
    )
:effect
  (and
    (increase (actual_deposit_weight ?r) (item_weight ?it))
    (gripper_free ?r)
    (store_item ?r ?it)
    (not (robot_carry ?r ?it))
  )
)

; Unload action. The robot unloads an object into the warehouse.
(:action unload
  :parameters (?it - item ?r - robot)
  :precondition 
    (and
      (store_item ?r ?it)
      (robot_at ?r Large-deposit)
    )
:effect
  (and
    (decrease (actual_deposit_weight ?r) (item_weight ?it))
    (item_in_deposit ?it)
    (not (store_item ?r ?it))
  )
)
)
