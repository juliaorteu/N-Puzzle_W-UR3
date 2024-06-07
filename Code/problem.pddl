
;; Eight puzzle problem #1.
;; Optimal solution cost = 57.

(define (problem korf1)
  (:domain strips-sliding-tile)
  (:objects t1 t2 t3 t4 t5 t6 t7 t8 
            p1 p2 p3
            rob)
  (:init
   (tile t1) (tile t2) (tile t3)
   (tile t4) (tile t5) (tile t6)
   (tile t7) (tile t8) 
   (position p1) (position p2) (position p3)
   (robot rob)
   (inc p1 p2) (inc p2 p3)
   (dec p3 p2) (dec p2 p1)
   (handEmpty rob)
   (at rob p3 p3)

   ;; initial state
   (at t4 p1 p1)
   (at t2 p2 p1)
   (at t5 p3 p1)
   (blank p1 p2)
   (at t6 p2 p2)
   (at t1 p3 p2)
   (at t7 p1 p3)
   (at t8 p2 p3)
   (at t3 p3 p3)
  )

  ;; standard goal state
  (:goal
   (and
    (at t1 p1 p1)
    (at t2 p2 p1)
    (at t3 p3 p1)
    (at t4 p1 p2)
    (at t5 p2 p2)
    (at t6 p3 p2)
    (at t7 p1 p3)
    (at t8 p2 p3)
    (blank p3 p3)
    (handEmpty rob)
   ))
 )
