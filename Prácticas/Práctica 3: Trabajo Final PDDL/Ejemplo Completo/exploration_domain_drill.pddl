(define (domain exploration_domain_drill)
(:requirements :strips :typing :numeric-fluents :equality :universal-preconditions)


; Types definition
(:types
  interest_point base - location
  robot
  sample
  soil
  tool
)

(:functions
  (robot_max_load ?r - robot)
  (sample_weight ?s - sample)
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (soil_at ?s - soil ?l - location)
  (tool_at ?t - tool ?l - location)
  (soil_analysed ?sl - soil)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?s - sample)
  (robot_carry_drill ?r - robot ?t - tool)
  (connected ?c1 ?c2 - location)
  (all_samples_at_base ?b - base)
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

(:action collect
  :parameters (?s - sample ?l - interest_point ?r - robot)
  :precondition 
    (and
      (sample_at ?s ?l)
      (robot_at ?r ?l)
      (gripper_free ?r)
      (>= (robot_max_load ?r) (sample_weight ?s))
    )
  :effect
    (and
      (robot_carry ?r ?s)
      (not (sample_at ?s ?l))
      (not (gripper_free ?r))
    )
)

(:action collect_drill
  :parameters (?t - tool ?l - location ?r - robot)
  :precondition 
    (and
      (tool_at ?t ?l)
      (robot_at ?r ?l)
      (gripper_free ?r)
    )
  :effect
    (and
      (robot_carry_drill ?r ?t)
      (not (tool_at ?t ?l))
      (not (gripper_free ?r))
    )
)

(:action drop
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (robot_carry ?r ?s)
    )
  :effect 
    (and 
      (sample_at ?s ?l)
      (gripper_free ?r)
      (not (robot_carry ?r ?s))
    )
)

(:action drop_drill
  :parameters (?t - tool ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (robot_carry_drill ?r ?t)
    )
  :effect 
    (and 
      (tool_at ?t ?l)
      (gripper_free ?r)
      (not (robot_carry_drill ?r ?t))
    )
)

(:action analyse-soil
  :parameters (?s - soil ?l - location ?r - robot ?t - tool)
  :precondition
    (and 
      (robot_at ?r ?l)
      (soil_at ?s ?l)
      (robot_carry_drill ?r ?t)
    )
  :effect 
    (soil_analysed ?s)
)

(:action check_all_samples_at_base
 :parameters (?b - base)
 :precondition
   (and
     (forall (?s - sample) (sample_at ?s ?b))
  )
 :effect
   (and
     (all_samples_at_base ?b)
   )

)
)