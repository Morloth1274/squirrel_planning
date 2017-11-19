(define (domain clear_path)
(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)

(:types
	waypoint robot object
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(connected ?wp ?wp2 - waypoint)
	(pushable_from ?wp ?wp2 - waypoint)
)

(:action make_connected
	:parameters(?wp ?wp2 - waypoint)
	:precondition(and
		(connected ?wp ?wp2)
	)
	:effect (and
		(connected ?wp2 ?wp)
	)
)

(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from)
		(connected ?from ?to)
	)
	:effect (and
		(not (robot_at ?v ?from))
		(robot_at ?v ?to)
	)
)

(:action push_object
	:parameters (?v - robot ?ob - object ?from ?to ?push_location ?object_location - waypoint)
	:precondition (and
		(robot_at ?v ?from)
		(object_at ?ob ?object_location)
		(connected ?from ?push_location)
		(connected ?object_location ?to)
		(pushable_from ?object_location ?push_location)
	)
	:effect (and
		;; For every state ?s
		;(not (robot_at ?v ?from))
		(not (object_at ?ob ?object_location))
		;(robot_at ?v ?to)
		(object_at ?ob ?to)
	)
)

)

