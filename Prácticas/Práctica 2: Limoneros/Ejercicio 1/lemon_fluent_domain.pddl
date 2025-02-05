(define (domain lemon_fluent)
(:requirements :strips :typing :fluents)

(:types
  location 
  robot  
  lemon
)

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

(:functions
  (lemon_weight ?l - lemon)
  (max_basket_capacity ?r - robot)
  (current_basket_weight ?r - robot)
)

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