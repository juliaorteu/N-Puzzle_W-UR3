(define (problem korf1)
  (:domain strips-sliding-tile)
  (:objects t1 t2 t3 t4 t5 t6 t7 t8
            p1 p2 p3)
  (:init
   (tile t1) (tile t2) (tile t3)
   (tile t4) (tile t5) (tile t6)
   (tile t7) (tile t8)
   (position p1) (position p2) (position p3)
   (inc p1 p2) (inc p2 p3)
   (dec p3 p2) (dec p2 p1)

   ;; initial state: first (top) row, left-to-right
   (at t1 p1 p1)
   (at t2 p2 p1)
   (at t3 p3 p1)
   ;; initial state: second row, left-to-right
   (at t4 p1 p2)
   (blank p2 p2)
   (at t5 p3 p2)
   ;; initial state: third row, left-to-right
   (at t6 p1 p3)
   (at t7 p2 p3)
   (at t8 p3 p3)
 )

 ;; standard goal state (but not mentioning blank)
 (:goal
  (and
   (at t1 p1 p1)
   (at t2 p2 p1)
   (at t3 p3 p1)
   (at t4 p1 p2)
   (at t5 p2 p2)
   (at t6 p3 p2)
   (at t7 p1 p3)
   (at t8 p2 p3)))
)

