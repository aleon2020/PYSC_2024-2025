(define (domain search_domain)
(:requirements :strips :typing :durative-actions)

(:types
  location
  drone
  person
)

(:predicates
  (drone_at ?r - drone ?l - location)
  (person_at ?p - person ?l - location)
  (person_found ?p - person)
  (connected ?l1 ?l2 - location)
)

; Fly action. The drone flies from one location (A) to another (B).
; The drone must be initially at the initial location.
; The 2 locations must be connected
; Consequence: The drone is now at B and not in A.
(:durative-action fly
  :parameters (?d - drone ?from ?to - location)
  :duration (= ?duration 4)
  :condition
    (and
      (at start (drone_at ?d ?from))
      (over all (connected ?from ?to))
    )
  :effect
    (and
      (at start (not (drone_at ?d ?from)))
      (at end (drone_at ?d ?to))
    )
)

; Search action. The drone finds a person.
; The drone and the person must be in the same location.
; The drone cannot move while searching.
; Consequences:
;     - The person is now found.
(:durative-action search
  :parameters (?d - drone ?p - person ?l - location)
  :duration (= ?duration 10)
  :condition
    (and
      (over all (drone_at ?d ?l))
      (over all (person_at ?p ?l))
    )
  :effect
    (and
      (at end (person_found ?p))
    )
)

)