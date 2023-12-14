(define (problem task)
    (:domain motion)
    (:objects
        rob - robot
        wp0 wp1 wp2 wp3 wp4 - waypoint
        m11 m12 m13 m15 - marker
    )
    (:init
        (robot_in rob wp0)
        (at_marker m11 wp1)
        (at_marker m12 wp2)
        (at_marker m13 wp3)
        (at_marker m15 wp4)
        (= (marker-counter) 0)
    )

    (:goal 
        (and
            (markerfound m11)
            (markerfound m12)
            (markerfound m13)
            (markerfound m15)
            (back_home wp0)
        )
    )
)
