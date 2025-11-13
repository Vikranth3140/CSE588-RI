(define (problem hybrid_pour_task)
  (:domain hybrid_domain)
  (:objects
    ur5 - robot
    pot mug kettle - object
    table counter sink shelf storage - location
  )
  (:init
    ; Robot initial position
    (at ur5 table)
    
    ; Location connectivity - symbolic graph says these are connected
    (connected table counter)
    (connected counter sink)
    (connected table shelf)
    (connected shelf storage)
    (connected counter storage)
    (connected sink storage)
    
    ; Symbolic reachability (appears feasible to discrete planner)
    (reachable table counter)
    (reachable counter sink)
    (reachable table shelf)
    (reachable shelf storage)
    (reachable counter storage)
    (reachable sink storage)
    
    ; However, NOT all paths are collision-free (geometric constraint)
    ; The discrete planner doesn't know this!
    (collision-free table counter)
    ; table->shelf has collision (narrow passage with obstacle)
    ; counter->storage has collision (object blocking path)
    (collision-free counter sink)
    (collision-free sink storage)
    
    ; Object locations
    (object-at pot table)
    (object-at mug counter)
    (object-at kettle shelf)
    
    ; Object states
    (clear pot)
    (clear mug)
    (clear kettle)
    
    ; Only some grasps are validated (geometric feasibility)
    (grasp-validated pot table)
    (grasp-validated mug counter)
    ; kettle grasp not validated - will cause replanning if attempted
    
    ; Distance metrics (used for battery consumption)
    (= (distance table counter) 0.8)
    (= (distance counter sink) 1.2)
    (= (distance table shelf) 0.5)
    (= (distance shelf storage) 1.5)
    (= (distance counter storage) 1.8)
    (= (distance sink storage) 0.6)
    
    ; Travel times
    (= (travel-time table counter) 3.0)
    (= (travel-time counter sink) 5.0)
    (= (travel-time table shelf) 2.0)
    (= (travel-time shelf storage) 6.0)
    (= (travel-time counter storage) 7.0)
    (= (travel-time sink storage) 3.0)
    
    ; Collision costs (higher = more likely to fail)
    (= (collision-cost table counter) 0.1)
    (= (collision-cost counter sink) 0.1)
    (= (collision-cost table shelf) 0.9)  ; High collision risk!
    (= (collision-cost shelf storage) 0.3)
    (= (collision-cost counter storage) 0.85)  ; High collision risk!
    (= (collision-cost sink storage) 0.2)
    
    ; Path clearance (lower = more constrained)
    (= (path-clearance table counter) 0.8)
    (= (path-clearance counter sink) 0.7)
    (= (path-clearance table shelf) 0.2)  ; Narrow!
    (= (path-clearance shelf storage) 0.5)
    (= (path-clearance counter storage) 0.15)  ; Very narrow!
    (= (path-clearance sink storage) 0.6)
    
    ; Battery state
    (= (battery-level ur5) 0.5)  ; Limited battery - forces careful planning
    (battery-ok ur5)
  )
  (:goal
    (and
      ; Primary goal: pour pot into mug
      (poured pot mug)
      ; Challenge: symbolic planner might try table->shelf->storage->counter
      ; but shelf path has collision, forcing replan through counter->sink->storage
    )
  )
  (:metric minimize (+ (* 10 (total-time)) 
                      (* 5 (- 1.0 (battery-level ur5)))))
)
