(define (domain airport-management)
(:requirements :durative-actions :typing :fluents)

; Types definition
(:types 
  airplane
  gate runway - location
  passenger_group
)

; Predicates definition
(:predicates
  (airplane_at ?a - airplane ?l - location)
  (airplane_flying ?a - airplane)
  (airplane_free ?a - airplane)
  (airplane_passengers ?a - airplane ?p - passenger_group)
  (gate_passengers ?g - gate ?p - passenger_group)
  (location_free ?l - location)
)
  
; Functions definition
(:functions
  (distance ?l1 - location ?l2 - location)
)

; Taxi action. The airplane moves from one location (A) to another (B).
; Conditions:
;   - The airplane must be at the starting location.
;   - The destination location must be free.
; Consequences:
;   - The airplane is now at B and not at A.
;   - The destination location is occupied, and the previous location is freed.
(:durative-action taxi
  :parameters (?a - airplane ?from - location ?to - location)
  :duration (= ?duration (distance ?from ?to))
  :condition 
    (and 
      (at start (airplane_at ?a ?from))
      (at start (location_free ?to))
    )
  :effect 
    (and 
      (at start (not (airplane_at ?a ?from)))
      (at start (not (location_free ?to)))
      (at end (airplane_at ?a ?to))
      (at end (location_free ?from))
    )
)

; Takeoff action. The airplane takes off from a runway.
; Conditions:
;   - The airplane must be at the runway.
;   - The airplane must have passengers.
; Consequences:
;   - The airplane is no longer at the runway.
;   - The airplane is now flying.
;   - The runway is freed.
(:durative-action takeoff
  :parameters (?a - airplane ?r - runway ?p - passenger_group)
  :duration (= ?duration 15)
  :condition 
    (and 
      (at start (airplane_at ?a ?r))
      (at start (airplane_passengers ?a ?p))
    )
  :effect 
    (and 
      (at start (not (airplane_at ?a ?r)))
      (at start (not (location_free ?r)))
      (at end (airplane_flying ?a))
      (at end (location_free ?r))
    )
)

; Land action. The airplane lands on a runway.
; Conditions:
;   - The airplane must be flying.
; Consequences:
;   - The airplane is no longer flying.
;   - The airplane is now at the runway.
;   - The runway is freed.
(:durative-action land
  :parameters (?a - airplane ?r - runway)
  :duration (= ?duration 20)
  :condition 
    (and
      (at start (airplane_flying ?a))
    )
  :effect 
    (and 
      (at start (not (airplane_flying ?a)))
      (at start (not (location_free ?r)))
      (at end (airplane_at ?a ?r))
      (at end (location_free ?r))
    )
)

; Board action. A group of passengers boards an airplane at a gate.
; Conditions:
;   - The airplane must be at the gate.
;   - The airplane must be free.
;   - Passengers must be waiting at the gate.
; Consequences:
;   - The passengers are no longer at the gate.
;   - The passengers are now aboard the airplane.
;   - The airplane is no longer free.
(:durative-action board
  :parameters (?a - airplane ?g - gate ?p - passenger_group)
  :duration (= ?duration 70)
  :condition 
    (and 
      (at start (airplane_at ?a ?g))
      (at start (airplane_free ?a))
      (at start (gate_passengers ?g ?p))
    )
  :effect 
    (and 
      (at start (not (gate_passengers ?g ?p)))
      (at end (airplane_passengers ?a ?p))
      (at end (not (airplane_free ?a)))
    )
)

; Disembark action. A group of passengers leaves the airplane at a gate.
; Conditions:
;   - The airplane must be at the gate.
; Consequences:
;   - The passengers are no longer aboard the airplane.
;   - The passengers are now at the gate.
;   - The airplane is free again.
(:durative-action disembark
  :parameters (?a - airplane ?g - gate ?p - passenger_group)
  :duration (= ?duration 70)
  :condition 
    (and
      (at start (airplane_at ?a ?g))
    )
  :effect 
    (and 
      (at start (not (airplane_passengers ?a ?p)))
      (at end (gate_passengers ?g ?p))
      (at end (airplane_free ?a))
    )
)

)