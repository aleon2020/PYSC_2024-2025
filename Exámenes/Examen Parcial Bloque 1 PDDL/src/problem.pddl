(define (problem airport-scenario)
(:domain airport-management)

(:objects
    airplane_1 - airplane
    airplane_2 - airplane
    airplane_3 - airplane
    airplane_4 - airplane
    airplane_5 - airplane

    runway_1 - runway
    gate_1 - gate
    gate_2 - gate
    gate_3 - gate

    passengers_1 - passenger_group
    passengers_2 - passenger_group
    passengers_3 - passenger_group
    passengers_4 - passenger_group
    passengers_5 - passenger_group
    passengers_6 - passenger_group
)

(:init
    ; Airplane 1 is at the runway ready to disembark the passenger group 1
    (airplane_at airplane_1 runway_1)
    (airplane_passengers airplane_1 passengers_1)

    ; Airplane 2 is free at gate 2
    (airplane_at airplane_2 gate_2)
    (airplane_free airplane_2)

    ; Airplane 3 is flying with passenger group 3
    (airplane_flying airplane_3)
    (airplane_passengers airplane_3 passengers_3)

    ; Airplane 4 is flying with passenger group 4
    (airplane_flying airplane_4)
    (airplane_passengers airplane_4 passengers_4)

    ; Airplane 5 is free at gate 1
    (airplane_at airplane_5 gate_1)
    (airplane_free airplane_5)

    ; Gate 3 does not have any airplane
    (location_free gate_3)

    ; Passenger group 2 is at gate 3
    (gate_passengers gate_3 passengers_2)
    ; Passenger group 5 is at gate 1
    (gate_passengers gate_1 passengers_5)
    ; Passenger group 6 is at gate 2
    (gate_passengers gate_2 passengers_6)

    ; Distance between locations
    (= (distance gate_1 gate_2) 10)
    (= (distance gate_2 gate_1) 10)
    (= (distance gate_1 gate_3) 15)
    (= (distance gate_3 gate_1) 15)
    (= (distance gate_2 gate_3) 5)
    (= (distance gate_3 gate_2) 5)

    (= (distance gate_1 runway_1) 30)
    (= (distance runway_1 gate_1) 30)
    (= (distance gate_2 runway_1) 30)
    (= (distance runway_1 gate_2) 30)
    (= (distance gate_3 runway_1) 30)
    (= (distance runway_1 gate_3) 30)

)

(:goal
    (and
        ; Passenger group 1 must be at gate 2
        (gate_passengers gate_2 passengers_1)
        ; Passenger group 2 must fly on airplane 2
        (airplane_passengers airplane_2 passengers_2)
        (airplane_flying airplane_2)
        ; Passenger group 3 must be at gate 1
        (gate_passengers gate_1 passengers_3)
        ; Passenger group 4 must be at gate 3
        (gate_passengers gate_3 passengers_4)
        ; Passenger group 5 must fly on airplane 5
        (airplane_passengers airplane_5 passengers_5)
        (airplane_flying airplane_5)
        ; Passenger group 6 must fly on airplane 1
        (airplane_passengers airplane_1 passengers_6)
        (airplane_flying airplane_1)
    )
)

)