(define (domain lemon_extra)
(:requirements :strips :typing :fluents :durative-actions)

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
  (locations_distance ?from ?to - location)
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration (locations_distance ?from ?to))
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