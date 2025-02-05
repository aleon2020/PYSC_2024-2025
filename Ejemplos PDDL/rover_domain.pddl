(define (domain rover)
(:requirements :typing :fluents)
(:types robot waypoint - object)
(:predicates
(robot_at ?r - robot ?wp - waypoint)
(connected ?c1 ?c2 - waypoint)
(visit ?wp - waypoint)
)
(:action move
:parameters (?r - robot ?from ?to - waypoint)
:precondition (and
(connected ?from ?to)
(robot_at ?r ?from)
)
:effect (and
(not (robot_at ?r ?from))
(robot_at ?r ?to)
(visit ?from)
(visit ?to)
)
)
)
