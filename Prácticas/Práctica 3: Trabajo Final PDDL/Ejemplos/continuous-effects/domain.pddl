(define (domain continuous-effects)
(:requirements :strips :typing :durative-actions :continuous-effects :numeric-fluents :equality)

(:types
  location
  robot
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (move_faster ?l - location)
)

(:functions
  (battery_level ?r - robot)
  (consumption ?r - robot)
  (distance ?from - location ?to - location) 
)

(:constants
  base - location
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration (distance ?from ?to))
  :condition
    (and 
      (at start (robot_at ?r ?from))
      (at start (< (consumption ?r) 50))
    )
  :effect
    (and
      (at end (robot_at ?r ?to))
      (decrease (battery_level ?r) #t)
      (increase (consumption ?r) (* 3 #t))
      (at start (not (robot_at ?r ?from)))
    )
)

(:durative-action run
  :parameters (?r - robot ?l - location)
  :duration (= ?duration 5)
  :condition
    (and 
      (over all (< (consumption ?r) 50))
      (over all (robot_at ?r ?l))
    )
  :effect
    (and
      (decrease (battery_level ?r) (* 5 #t))
      (increase (consumption ?r) (* 3 #t))
    )
)

(:durative-action drink
  :parameters (?r - robot ?l - location)
  :duration (= ?duration 2)
  :condition
    (and 
      (over all (not(= ?l base)))
      (over all (robot_at ?r ?l))
    )
  :effect
    (and
      (decrease (consumption ?r) (* 5 #t))
      (increase (battery_level ?r) (* 2 #t))
    )
)

)