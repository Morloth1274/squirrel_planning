(define (problem squirrel_robot_behaviour_task)
(:domain squirrel_robot_behaviour)
(:objects
    c1 c2 c3 - child
    blue_oxygen red_oxygen green_oxygen blue_battery red_battery green_battery - object
    robot - robot
    oxygen battery - type
    blue_oxygen_location near_blue_oxygen red_oxygen_location near_red_oxygen green_oxygen_location near_green_oxygen blue_battery_location near_blue_battery red_battery_location near_red_battery green_battery_location near_green_battery kenny_waypoint - waypoint
)
(:init
    (battery_available blue_battery)
    (battery_available red_battery)
    (battery_available green_battery)
    (gripper_empty robot)
    (is_of_type blue_oxygen oxygen)
    (is_of_type red_oxygen oxygen)
    (is_of_type green_oxygen oxygen)
    (is_of_type blue_battery battery)
    (is_of_type red_battery battery)
    (is_of_type green_battery battery)
    (not_busy)
    (object_at blue_oxygen blue_oxygen_location)
    (object_at red_oxygen red_oxygen_location)
    (object_at green_oxygen green_oxygen_location)
    (object_at blue_battery blue_battery_location)
    (object_at red_battery red_battery_location)
    (object_at green_battery green_battery_location)
    (robot_at robot kenny_waypoint)
    (= (power robot) 4.5)
)
(:goal (and
    (child_has_oxygen c1)
    (child_has_oxygen c2)
    (child_has_oxygen c3)
)))
