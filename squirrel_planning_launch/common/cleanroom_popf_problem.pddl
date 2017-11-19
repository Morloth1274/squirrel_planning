(define (problem clear_path01)
(:domain clear_path)
(:objects
    kenny - robot
    wp1 wp2 wp3 push_wp object_wp - waypoint

    toy1 - object
)
(:init
    (robot_at kenny wp1)
    (pushable_from object_wp push_wp)
    (connected wp1 push_wp)
    (connected object_wp wp2)
    (connected object_wp wp3)
;    (connected wp3 wp2)

    (object_at toy1 object_wp)
)
(:goal (and
   (robot_at kenny wp2)
   (object_at toy1 wp3)
)))
