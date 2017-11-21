(define (problem clear_path01)
(:domain clear_path)
(:objects
    kenny - robot
    wp1 wp2 object_n_wp object_w_wp object_e_wp object_s_wp object_wp - waypoint
    toy1 - object
)
(:init
    (robot_at kenny wp1)
    (pushable_from object_wp object_n_wp)
    (pushable_from object_wp object_w_wp)
    (pushable_from object_wp object_e_wp)
    (pushable_from object_wp object_s_wp)
    (connected wp1 object_n_wp)
    (connected object_wp wp2)

    (object_at toy1 object_wp)
)
(:goal (and
;   (robot_at kenny wp2)
   (object_at toy1 wp2)
)))
