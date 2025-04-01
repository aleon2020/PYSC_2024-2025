; robot_domain.pddl
(define (domain robot_library)
(:requirements :strips :typing :negative-preconditions :fluents :durative-actions)

(:types 
  hall table bookshelf - location
  book miscellaneus - prop
  visitor
  robot
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (gripper_free ?r - robot)
  (object_at ?o - prop ?l - location)
  (visitor_at ?v - visitor ?l - location)
  (solved ?m - miscellaneus)
  (book_found ?b - book ?l - bookshelf)
  (noise_at ?l - location)
  (quiet ?l - location)
  (connected ?l1 ?l2 - location)
  (robot_not_busy ?r - robot)
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 2)
  :condition
    (and 
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and
      (at start (not (robot_not_busy ?r)))
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (robot_not_busy ?r))
    )
)

(:durative-action solve
  :parameters (?r - robot ?m - miscellaneus ?l - location)
  :duration (= ?duration 2)
  :condition 
    (and 
      (over all (robot_at ?r ?l))
      (over all (object_at ?m ?l))
      (at start (robot_not_busy ?r))
      (at start (gripper_free ?r))
    )
  :effect 
    (and
      (at start (not (robot_not_busy ?r)))
      (at start (not (gripper_free ?r))) 
      (at end (gripper_free ?r))
      (at end (solved ?m)) 
      (at end (robot_not_busy ?r))
    )
)

(:durative-action guide_visitor
  :parameters (?r - robot ?v - visitor ?from ?to - location)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (robot_at ?r ?from)) 
      (at start (visitor_at ?v ?from))
      (over all (connected ?from ?to))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and 
      (at start (not (robot_not_busy ?r)))
      (at start (not (visitor_at ?v ?from))) 
      (at start (not (robot_at ?r ?from)))
      (at end (visitor_at ?v ?to))
      (at end (robot_at ?r ?to))
      (at end (robot_not_busy ?r))
    )
)

(:durative-action search_book
  :parameters (?r - robot ?b - book ?l - bookshelf)
  :duration (= ?duration 2)
  :condition 
    (and 
      (over all (robot_at ?r ?l))
      (over all (object_at ?b ?l))
      (at start (gripper_free ?r))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and 
      (at start (not (robot_not_busy ?r)))
      (at end (book_found ?b ?l))
      (at end (robot_not_busy ?r))
    )
)

(:durative-action move_object
  :parameters (?r - robot ?o - prop ?from ?to - location)
  :duration (= ?duration 2)
  :condition 
    (and
      (at start (robot_at ?r ?from))
      (at start (object_at ?o ?from))
      (over all (connected ?from ?to))
      (at start (gripper_free ?r))
      (at start (robot_not_busy ?r))
    )
  :effect
    (and
      (at start (not (robot_not_busy ?r)))
      (at start (not (object_at ?o ?from)))
      (at start (not (gripper_free ?r)))
      (at start (not (robot_at ?r ?from)))
      (at end (object_at ?o ?to))
      (at end (gripper_free ?r))
      (at end (robot_not_busy ?r))
      (at end (robot_at ?r ?to))
    )
)

(:durative-action shut_up
  :parameters (?r - robot ?l - location)
  :duration (= ?duration 2)
  :condition 
    (and 
      (over all (robot_at ?r ?l))
      (at start (noise_at ?l))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and 
      (at start (not (robot_not_busy ?r)))
      (at start (not (noise_at ?l)))
      (at end (quiet ?l))
      (at end (robot_not_busy ?r))
    )
)

)