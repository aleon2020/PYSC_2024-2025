(define (domain lemon_fluent)
(:requirements :strips :typing :fluents)

; Types definition
(:types
  location 
  robot  
  lemon
)

; New location
(:constants
  Large-deposit - location
)

(:predicates
  (robot_at ?r - robot ?loc - location)
  (lemon_at ?l - lemon ?loc - location)
  (gripper_free ?r - robot)
  (holding ?r - robot ?l - lemon)
  (store_lemon ?r - robot ?l - lemon)
  (lemon_in_deposit ?l - lemon)
  (connected ?from ?to - location)
)

; Functions
(:functions
  (lemon_weight ?l - lemon)
  (max_basket_capacity ?r - robot)
  (current_basket_weight ?r - robot)
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:action move
  :parameters (?r - robot ?from ?to - location)
  :precondition 
    (and 
      (robot_at ?r ?from) 
      (connected ?from ?to)
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
; - The item is no longer at the given location.
; - The robot is now carrying the object and its gripper is not free.
(:action pick
  :parameters (?l - lemon ?loc - location ?r - robot)
  :precondition 
    (and 
      (lemon_at ?l ?loc)
      (robot_at ?r ?loc) 
      (gripper_free ?r)
    )
  :effect 
    (and 
      (holding ?r ?l) 
      (not (lemon_at ?l ?loc))
      (not (gripper_free ?r))
    )
)

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
; - The item is now at the given location.
; - The robot is no longer carrying the object and its gripper is free.
(:action drop
  :parameters (?l - lemon ?loc - location ?r - robot)
  :precondition 
    (and 
      (robot_at ?r ?loc)
      (holding ?r ?l) 
    )
  :effect 
    (and 
      (lemon_at ?l ?loc)
      (gripper_free ?r) 
      (not (holding ?r ?l))
    )
)

; Deposit action. The robot loads an object into its container. 
; The object must be grasped by the gripper.
(:action deposit
  :parameters (?l - lemon ?r - robot)
  :precondition 
    (and 
      (> (max_basket_capacity ?r) (+ (current_basket_weight ?r) (lemon_weight ?l)))
      (holding ?r ?l) 
    )
  :effect 
    (and 
      (increase (current_basket_weight ?r) (lemon_weight ?l))
      (gripper_free ?r) 
      (store_lemon ?r ?l)
      (not (holding ?r ?l))
    )
)

; Unload action. The robot unloads an object into the base.
(:action unload
    :parameters (?l - lemon ?r - robot)
    :precondition 
      (and 
        (store_lemon ?r ?l)
        (robot_at ?r Large-deposit) 
      )
    :effect 
      (and
        (decrease (current_basket_weight ?r) (lemon_weight ?l))
        (lemon_in_deposit ?l)
        (not (store_lemon ?r ?l))
      )
)

)