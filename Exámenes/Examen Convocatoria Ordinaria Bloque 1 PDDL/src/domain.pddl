(define (domain heist_domain)
(:requirements :typing :durative-actions :equality)
  
; Types definition
(:types
  robot
  location waypoint control_room vault - location
  camera
  artifact
)

; Predicates definition
(:predicates
  (robot_at ?r - robot ?l - location)
  (is_hacker ?r - robot)
  (is_lifter ?r - robot)
  (connected ?from ?to - location)
  (has_door ?from ?to - location)
  (camera_in ?c - camera ?l - location)
  (has_clearance ?l - location)
  (control_available ?c - control_room)
  (artifact_in ?a - artifact ?l - location)
  (carry ?r - robot ?a - artifact)
  (safe_opened ?l - vault)
)

; Move action. The robot moves from one location to another.
; Conditions:
;   - The robot must be at the starting location.
;   - The locations must be connected.
;   - The destination must have clearance.
; Consequences:
;   - The robot is no longer at the starting location.
;   - The robot is now at the destination location.
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 5)
  :condition
    (and
      (at start (robot_at ?r ?from))
      (at start (connected ?from ?to))
      (at start (has_clearance ?to))
    )
  :effect
    (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
)

; Open Door action. Two robots work together to open a door between locations.
; Conditions:
;   - There must be a door between the locations.
;   - Two different robots must be at the starting location.
; Consequences:
;   - The locations become connected in both directions.
(:durative-action open_door
  :parameters (?r1 ?r2 - robot ?from ?to - location)
  :duration (= ?duration 2)
  :condition
    (and
      (over all (has_door ?from ?to))
      (over all (robot_at ?r1 ?from))
      (over all (robot_at ?r2 ?from))
      (over all (not (= ?r1 ?r2)))
    )
  :effect
    (and
      (at end (connected ?from ?to))
      (at end (connected ?to ?from))
    )
)

; Disable Camera action. A hacker robot disables a camera from the control room.
; Conditions:
;   - The robot must be a hacker.
;   - The control room must be available.
;   - The robot must be in the control room.
;   - The camera must be in the target location.
; Consequences:
;   - The target location gains clearance.
(:durative-action disable_camera
  :parameters (?r - robot ?c - control_room ?cam - camera ?l - location)
  :duration (= ?duration 3)
  :condition
    (and
      (over all (is_hacker ?r))
      (over all (control_available ?c))
      (over all (robot_at ?r ?c))
      (at start (camera_in ?cam ?l))
    )
  :effect
    (at end (has_clearance ?l))
)

; Open Safe action. A hacker robot opens a vault safe.
; Conditions:
;   - The robot must be a hacker.
;   - The robot must be in the vault.
; Consequences:
;   - The vault safe is opened.
(:durative-action open_safe
  :parameters (?r - robot ?v - vault)
  :duration (= ?duration 15)
  :condition
    (and
      (over all (is_hacker ?r))
      (over all (robot_at ?r ?v))
    )
  :effect
    (at end (safe_opened ?v))
)

; Take Artifact action. A lifter robot picks up an artifact from an opened vault.
; Conditions:
;   - The robot must be a lifter.
;   - The robot must be in the vault.
;   - The artifact must be in the vault.
;   - The vault safe must be opened.
; Consequences:
;   - The robot is now carrying the artifact.
(:durative-action take_artifact
  :parameters (?r - robot ?a - artifact ?v - vault)
  :duration (= ?duration 1)
  :condition
    (and
      (at start (is_lifter ?r))
      (at start (robot_at ?r ?v))
      (at start (artifact_in ?a ?v))
      (at start (safe_opened ?v))
    )
  :effect
    (at end (carry ?r ?a))
)

)