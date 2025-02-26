(define (domain airport-management)
(:requirements :durative-actions :typing :fluents)

; Types definition
(:types 
  airplane location passenger_group - object
  gate runway - location
)

; Predicates definition
(:predicates
  (airplane_at ?a - airplane ?l - location)
  (airplane_flying ?a - airplane)
  (location_free ?l - location)
  (airplane_free ?a - airplane)
  (airplane_passengers ?a - airplane ?p - passenger_group)
  (gate_passengers ?g - gate ?p - passenger_group)
)
  
; Functions definition
(:functions
  (distance ?l1 ?l2 - location)
)

; Taxi action. The airplane moves from one location (A) to another (B).
; Conditions:
;   - The airplane must be at the starting location.
;   - The destination location must be free.
; Consequences:
;   - The airplane is now at B
;   - The airplane is not at A.
;   - The destination location is occupied
;   - The previous location is freed.
(:durative-action taxi
  :parameters (?a - airplane ?from - location ?to - location)
  :duration (= ?duration (* (distance ?from ?to) 2))
  :condition 
    (and 
      (at start (airplane_at ?a ?from))
      (at start (location_free ?to))
    )
  :effect 
    (and 
      (at start (not (location_free ?to)))
      (at start (not (airplane_at ?a ?from)))
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
;   - The runway is not freed.
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
;   - The runway is freed.
;   - The airplane must be flying.
; Consequences:
;   - The runway is not freed.
;   - The airplane is no longer flying.
;   - The airplane is now at the runway.
(:durative-action land
  :parameters (?a - airplane ?r - runway)
  :duration (= ?duration 20)
  :condition 
    (and
      (at start (location_free ?r))
      (at start (airplane_flying ?a))
    )
  :effect 
    (and 
      (at start (not (location_free ?r)))
      (at start (not (airplane_flying ?a)))
      (at end (airplane_at ?a ?r))
    )
)

; Board action. A group of passengers boards an airplane at a gate.
; Conditions:
;   - The airplane must be at the gate.
;   - Passengers must be waiting at the gate.
;   - The airplane must be free.
; Consequences:
;   - The passengers are no longer at the gate.
;   - The passengers are now aboard the airplane.
;   - The airplane is no longer free.
(:durative-action board
  :parameters (?a - airplane ?g - gate ?p - passenger_group)
  :duration (= ?duration 70)
  :condition 
    (and 
      (over all (airplane_at ?a ?g))
      (at start (gate_passengers ?g ?p))
      (at start (airplane_free ?a))
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
;   - The passengers are now aboard the airplane.
; Consequences:
;   - The passengers are now at the gate.
;   - The passengers are no longer aboard the airplane.
;   - The airplane is free again.
(:durative-action disembark
  :parameters (?a - airplane ?g - gate ?p - passenger_group)
  :duration (= ?duration 70)
  :condition 
    (and
      (over all (airplane_at ?a ?g))
      (at start (airplane_passengers ?a ?p))
    )
  :effect 
    (and 
      (at start (gate_passengers ?g ?p))
      (at end (not (airplane_passengers ?a ?p)))
      (at end (airplane_free ?a))
    )
)

)