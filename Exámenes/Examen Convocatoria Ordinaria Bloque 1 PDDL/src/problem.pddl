(define (problem heist_problem)
(:domain heist_domain)

(:objects
    hacker_bot cage_bot - robot
    outside entrance hallway - waypoint
    control - control_room
    secret_vault - vault
    cam_hallway cam_vault - camera
    declaration_of_independence - artifact
)

(:init
    (is_hacker hacker_bot)
    (is_lifter cage_bot)
    (robot_at hacker_bot outside)
    (robot_at cage_bot outside)

    (connected outside entrance)
    (connected entrance outside)
    (connected entrance control)
    (connected control entrance)
    ; There is a closed door between entrance and hallway. Not connected yet.
    (has_door entrance hallway)
    (has_door hallway entrance)
    ; There is a closed door between hallway and the vault. Not connected yet.
    (has_door hallway secret_vault)
    (has_door secret_vault hallway)

    ; These areas have cameras
    (camera_in cam_hallway hallway)
    (camera_in cam_vault secret_vault)

    ; These areas are clear and have no cameras. Robots can move freely.
    (has_clearance outside)
    (has_clearance entrance)
    (has_clearance control)

    ; The control room is initially available and can be used
    (control_available control)

    ; The declaration of independence is in the secret vault
    (artifact_in declaration_of_independence secret_vault)
)

; Both robots must be outside with the declaration of independence
(:goal
    (and
        (carry cage_bot declaration_of_independence)
        (robot_at hacker_bot outside)
        (robot_at cage_bot outside)
    )
)
)
