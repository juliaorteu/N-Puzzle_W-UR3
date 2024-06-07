(define (domain strips-sliding-tile)
  (:requirements :strips)
  (:predicates
   (tile ?x) (position ?x)
   (at ?t ?x ?y) (blank ?x ?y)
   (inc ?p ?pp) (dec ?p ?pp)
   (handEmpty ?rob)
   (holding ?rob ?t)
   (robot ?rob))

  (:action move-up
    :parameters (?t ?px ?py ?by ?rob)
    :precondition (and
           (tile ?t) (position ?px) (position ?py) (position ?by) (robot ?rob) (holding ?rob ?t)
           (dec ?by ?py) (blank ?px ?by) (at ?t ?px ?py) (at ?rob ?px ?py)
           )
    :effect (and (not (blank ?px ?by)) (not (at ?t ?px ?py)) (not (at ?rob ?px ?py))
         (blank ?px ?py) (at ?t ?px ?by) (at ?rob ?px ?by)))

  (:action move-down
    :parameters (?t ?px ?py ?by ?rob)
    :precondition (and
           (tile ?t) (position ?px) (position ?py) (position ?by) (robot ?rob) (holding ?rob ?t) 
           (inc ?by ?py) (blank ?px ?by) (at ?t ?px ?py) (at ?rob ?px ?py)
           )
    :effect (and (not (blank ?px ?by)) (not (at ?t ?px ?py)) (not (at ?rob ?px ?py))
         (blank ?px ?py) (at ?t ?px ?by) (at ?rob ?px ?by)))

  (:action move-left
    :parameters (?t ?px ?py ?bx ?rob)
    :precondition (and
           (tile ?t) (position ?px) (position ?py) (position ?bx) (robot ?rob) (holding ?rob ?t)
           (dec ?bx ?px) (blank ?bx ?py) (at ?t ?px ?py) (at ?rob ?px ?py)
           )
    :effect (and (not (blank ?bx ?py)) (not (at ?t ?px ?py)) (not (at ?rob ?px ?py))
         (blank ?px ?py) (at ?t ?bx ?py) (at ?rob ?bx ?py)))

  (:action move-right
    :parameters (?t ?px ?py ?bx ?rob)
    :precondition (and
           (tile ?t) (position ?px) (position ?py) (position ?bx) (robot ?rob) (holding ?rob ?t)
           (inc ?bx ?px) (blank ?bx ?py) (at ?t ?px ?py) (at ?rob ?px ?py)
           )
    :effect (and (not (blank ?bx ?py)) (not (at ?t ?px ?py)) (not (at ?rob ?px ?py))
         (blank ?px ?py) (at ?t ?bx ?py) (at ?rob ?bx ?py)))
  

(:action pick
:parameters (?rob ?t ?x ?y)
:precondition (and (tile ?t) (position ?x) (position ?y) (robot ?rob) (handEmpty ?rob) (at ?t ?x ?y) (at ?rob ?x ?y))
:effect (and (holding ?rob ?t)
   (not (handEmpty ?rob)) ))

(:action place
:parameters (?rob ?t ?x ?y)
:precondition (and (tile ?t) (position ?x) (position ?y) (robot ?rob) (holding ?rob ?t) (at ?t ?x ?y) (at ?rob ?x ?y)  )
:effect (and (handEmpty ?rob)
   (not (holding ?rob ?t)) ))

(:action move
:parameters (?rob ?x1 ?y1 ?x2 ?y2)
:precondition (and  (position ?x1) (position ?y1) (position ?x2) (position ?y2) (robot ?rob) (handEmpty ?rob) (at ?rob ?x1 ?y1)  )
:effect (and (at ?rob ?x2 ?y2) (not (at ?rob ?x1 ?y1)) ))

   )
