(define (domain lemon_durative)
(:requirements :strips :typing :fluents :durative-actions)

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
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 10)
  :condition 
    (and 
      (at start (robot_at ?r ?from)) 
      (at start (connected ?from ?to))
    )
  :effect 
    (and 
      (at end (robot_at ?r ?to))
      (at start (not (robot_at ?r ?from)))
    )
)

; Pick-up action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
; - The item is no longer at the given location.
; - The robot is now carrying the object and its gripper is not free.
(:durative-action pick
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
      (at end (holding ?r ?l))
      (at start (not (lemon_at ?l ?loc)))
      (at start (not (gripper_free ?r)))
    )
)

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
; - The item is now at the given location.
; - The robot is no longer carrying the object and its gripper is free.
(:durative-action drop
  :parameters (?l - lemon ?loc - location ?r - robot)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (robot_at ?r ?loc))
      (at start (holding ?r ?l))
    )
  :effect 
    (and 
      (at start (lemon_at ?l ?loc))
      (at start (gripper_free ?r))
      (at start (not (holding ?r ?l)))
    )
)

; Deposit action. The robot loads an object into its container. 
; The object must be grasped by the gripper.
(:durative-action deposit
  :parameters (?l - lemon ?r - robot)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (> (max_basket_capacity ?r) (+ (current_basket_weight ?r) (lemon_weight ?l))))
      (at start (holding ?r ?l))
    )
  :effect 
    (and 
      (at end (increase (current_basket_weight ?r) (lemon_weight ?l)))
      (at end (gripper_free ?r))
      (at end (store_lemon ?r ?l))
      (at start (not (holding ?r ?l)))
    )
)

; Unload action. The robot unloads an object into the base.
(:durative-action unload
    :parameters (?l - lemon ?r - robot)
    :duration (= ?duration 5)
    :condition 
      (and 
        (at start (store_lemon ?r ?l))
        (at start (robot_at ?r Large-deposit))
      )
    :effect 
      (and
        (at end (decrease (current_basket_weight ?r) (lemon_weight ?l)))
        (at end (lemon_in_deposit ?l))
        (at start (not (store_lemon ?r ?l)))
      )
)

)