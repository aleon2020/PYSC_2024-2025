(define (problem hanoi3)

(:domain hanoi)

(:objects 
  peg1 
  peg2 
  peg3 
  d1 
  d2 
  d3
)

(:init
  (= (size peg1) 0)
  (= (size peg2) 0)
  (= (size peg3) 0)
  (= (size d1) 3)
  (= (size d2) 2)
  (= (size d3) 1)
  (clear peg2) 
  (clear peg3) 
  (clear d1)
  (on d3 peg1) 
  (on d2 d3) 
  (on d1 d2)
)

(:goal (and 
  (on d3 peg3) 
  (on d2 d3) 
  (on d1 d2)
  )
)

)
