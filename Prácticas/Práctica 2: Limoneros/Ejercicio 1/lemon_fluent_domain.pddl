(define (domain lemon_fluent)
(:requirements :strips :typing :fluents)

; Types definition
(:types
  location 
  robot  
  lemon
  basket
)

(:predicates
  (robot_at ?r - robot ?loc - location)
  (lemon_at ?l - lemon ?loc - location)
  (basket_at ?b - basket ?loc - location)
  (gripper_free ?r - robot)
  (lemon_carry ?r - robot ?l - lemon)
  (basket_carry ?r - robot ?b - basket)
  (lemon_in_basket ?l - lemon ?b - basket)
  (connected ?from ?to - location)
  (lemon_tree ?loc - location)
)

; Functions
(:functions
  (lemon_weight ?l - lemon)
  (max_basket_capacity ?b - basket)
  (current_basket_weight ?b - basket)
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:action move
  :parameters (?r - robot ?from ?to - location ?b - basket)
  :precondition 
    (and 
      (robot_at ?r ?from) 
      (connected ?from ?to)
      (basket_carry ?r ?b)
    )
  :effect 
    (and 
      (robot_at ?r ?to) 
      (not (robot_at ?r ?from))
    )
)

; Pick-up lemon action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
; - The item is no longer at the given location.
; - The robot is now carrying the object and its gripper is not free.
(:action pick-lemon
  :parameters (?l - lemon ?loc - location ?r - robot)
  :precondition 
    (and 
      (lemon_at ?l ?loc)
      (robot_at ?r ?loc) 
      (gripper_free ?r)
    )
  :effect 
    (and 
      (lemon_carry ?r ?l) 
      (not (lemon_at ?l ?loc))
      (not (gripper_free ?r))
    )
)

; Pick-up basket action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
; - The item is no longer at the given location.
; - The robot is now carrying the object and its gripper is not free.
(:action pick-basket
  :parameters (?b - basket ?loc - location ?r - robot)
  :precondition 
    (and 
      (basket_at ?b ?loc)
      (robot_at ?r ?loc) 
      (gripper_free ?r)
    )
  :effect 
    (and 
      (basket_carry ?r ?b) 
      (not (basket_at ?b ?loc))
      (not (gripper_free ?r))
    )
)

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
; - The item is now at the given location.
; - The robot is no longer carrying the object and its gripper is free.
(:action drop
  :parameters (?b - basket ?loc - location ?r - robot)
  :precondition 
    (and 
      (robot_at ?r ?loc)
      (basket_carry ?r ?b)
      (lemon_tree ?loc)
    )
  :effect 
    (and 
      (basket_at ?b ?loc)
      (gripper_free ?r) 
      (not (basket_carry ?r ?b))
    )
)

; Deposit action. The robot loads an object into its container. 
; The object must be grasped by the gripper.
(:action deposit
  :parameters (?r - robot ?loc - location ?l - lemon ?b - basket)
  :precondition 
    (and 
      (lemon_carry ?r ?l) 
      (robot_at ?r ?loc)
      (basket_at ?b ?loc)
      (<= (+ (current_basket_weight ?b) (lemon_weight ?l)) (max_basket_capacity ?b))
    )
  :effect 
    (and 
      (increase (current_basket_weight ?b) (lemon_weight ?l))
      (gripper_free ?r) 
      (lemon_in_basket ?l ?b)
      (not (lemon_carry ?r ?l))
    )
)

; Unload action. The robot unloads an object into the base.
(:action unload
    :parameters (?r - robot ?loc - location ?b - basket ?l - lemon)
    :precondition 
      (and 
        (basket_carry ?r ?b)
        (lemon_in_basket ?l ?b)
        (robot_at ?r ?loc) 
      )
    :effect 
      (and
        (not (lemon_in_basket ?l ?b))
        (lemon_at ?l ?loc)
        (assign (current_basket_weight ?b) 0)
      )
)

)