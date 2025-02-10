(define (domain lemon_durative)
(:requirements :strips :typing :fluents :durative-actions)

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
(:durative-action move
  :parameters (?r - robot ?from ?to - location ?b - basket)
  :duration (= ?duration 10)
  :condition 
    (and 
      (at start (robot_at ?r ?from)) 
      (at start (connected ?from ?to))
      (at start (basket_carry ?r ?b))
    )
  :effect 
    (and 
      (at end (robot_at ?r ?to))
      (at start (not (robot_at ?r ?from)))
    )
)

; Pick-up lemon action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
; - The item is no longer at the given location.
; - The robot is now carrying the object and its gripper is not free.
(:durative-action pick-lemon
  :parameters (?l - lemon ?loc - location ?r - robot)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (lemon_at ?l ?loc))
      (at start (robot_at ?r ?loc))
      (at start (gripper_free ?r))
    )
  :effect 
    (and 
      (at end (lemon_carry ?r ?l))
      (at start (not (lemon_at ?l ?loc)))
      (at start (not (gripper_free ?r)))
    )
)

; Pick-up basket action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
; - The item is no longer at the given location.
; - The robot is now carrying the object and its gripper is not free.
(:durative-action pick-basket
  :parameters (?b - basket ?loc - location ?r - robot)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (basket_at ?b ?loc))
      (at start (robot_at ?r ?loc))
      (at start (gripper_free ?r))
    )
  :effect 
    (and 
      (at end (basket_carry ?r ?b))
      (at start (not (basket_at ?b ?loc)))
      (at start (not (gripper_free ?r)))
    )
)

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
; - The item is now at the given location.
; - The robot is no longer carrying the object and its gripper is free.
(:durative-action drop
  :parameters (?b - basket ?loc - location ?r - robot)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (robot_at ?r ?loc))
      (at start (basket_carry ?r ?b))
      (at start (lemon_tree ?loc))
    )
  :effect 
    (and 
      (at start (basket_at ?b ?loc))
      (at start (gripper_free ?r))
      (at start (not (basket_carry ?r ?b)))
    )
)

; Deposit action. The robot loads an object into its container. 
; The object must be grasped by the gripper.
(:durative-action deposit
  :parameters (?r - robot ?loc - location ?l - lemon ?b - basket)
  :duration (= ?duration 5)
  :condition 
    (and 
      (at start (lemon_carry ?r ?l))
      (at start (robot_at ?r ?loc))
      (at start (basket_at ?b ?loc))
      (at start (<= (+ (current_basket_weight ?b) (lemon_weight ?l)) (max_basket_capacity ?b)))
    )
  :effect 
    (and 
      (at end (increase (current_basket_weight ?b) (lemon_weight ?l)))
      (at end (gripper_free ?r))
      (at end (lemon_in_basket ?l ?b))
      (at start (not (lemon_carry ?r ?l)))
    )
)

; Unload action. The robot unloads an object into the base.
(:durative-action unload
    :parameters (?r - robot ?loc - location ?b - basket ?l - lemon)
    :duration (= ?duration 5)
    :condition 
      (and 
        (at start (basket_carry ?r ?b))
        (at start (lemon_in_basket ?l ?b))
        (at start (robot_at ?r ?loc))
      )
    :effect 
      (and
        (at start (not (lemon_in_basket ?l ?b)))
        (at end (lemon_at ?l ?loc))
        (at end (assign (current_basket_weight ?b) 0))
      )
)

)