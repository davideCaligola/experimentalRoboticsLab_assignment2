(define (domain motion)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents)


(:types 	robot waypoint marker)


(:predicates
		(robot_in ?r - robot ?wp - waypoint)
		(visited ?wp - waypoint)
		(markerfound ?m - marker)
		(at_marker ?m - marker ?wp - waypoint)
		(back_home ?wp - waypoint)
	      
)

(:functions      
		(marker-counter)
)


(:durative-action goto
		:parameters (?r - robot ?from ?to - waypoint)
		:duration (= ?duration 30)
		:condition (and (at start (robot_in ?r ?from)))
	        :effect (and 
	        		(at start (not (robot_in ?r ?from))) 
				(at end (robot_in ?r ?to)))
)

(:durative-action search
                :parameters (?r - robot ?wp - waypoint ?m - marker)
		:duration (= ?duration 10)
		:condition (and (at start (robot_in ?r ?wp))
				(at start (at_marker ?m ?wp)))
	        :effect (and 
				(at end (markerfound ?m))
				(at end (increase (marker-counter) 1)))
)

(:durative-action go-home
		:parameters (?r - robot ?from - waypoint ?to - waypoint)
		:duration (= ?duration 30)
		:condition (and (at start (robot_in ?r ?from))
				(at start (= (marker-counter) 4)))

	        :effect (and 
				(at start (not (robot_in ?r ?from))) 
				(at end (robot_in ?r ?to))
				(at end (back_home ?to))))



)
