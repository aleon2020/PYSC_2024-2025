(define (domain hanoi)
(:requirements :strips :fluents)
(:functions
(size ?x)
)
(:predicates
(clear ?x)
(on ?x ?y)
)
(:action move
:parameters (?disc ?from ?to)
:precondition (and
(< (size ?to) (size ?disc))
(on ?disc ?from)
(clear ?disc)
(clear ?to)
)
:effect (and
(clear ?from)
(on ?disc ?to)
(not (on ?disc ?from))
(not (clear ?to))
)
)
)
