(define (problem hybrid_pour_task)
  (:domain hybrid_domain)
  (:objects
    ur5 - robot
    pot mug - object
    table counter sink - location
  )
  (:init
    (at ur5 table)
    (connected table counter)
    (connected counter sink)
    (reachable table counter)
    (reachable counter sink)
    (clear pot)
    (= (distance table counter) 0.8)
    (= (battery-level ur5) 1.0)
    (battery-ok ur5)
  )
  (:goal
    (and
      (poured pot mug)
    )
  )
)
