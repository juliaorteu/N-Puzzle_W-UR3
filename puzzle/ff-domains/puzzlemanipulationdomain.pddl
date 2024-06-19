(define (domain puzzlemanipulationdomain)

(:types obstacle robot location)

(:predicates
       (handEmpty ?rob)
       (holding ?rob ?obs)
       (in ?obs ?from)
       (empty ?location))

(:action pick
:parameters (?rob - robot ?obs - obstacle ?from - location)
:precondition (and (handEmpty ?rob) (in ?obs ?from))
:effect (and (holding ?rob ?obs)
   (not (handEmpty ?rob)) (empty ?from) ))

(:action place
:parameters (?rob - robot ?obs - obstacle ?from - location)
:precondition (and (holding ?rob ?obs) (empty ?from))
:effect (and (handEmpty ?rob) (in ?obs ?from)
   (not (holding ?rob ?obs)) (not (empty ?from)) ))
)
