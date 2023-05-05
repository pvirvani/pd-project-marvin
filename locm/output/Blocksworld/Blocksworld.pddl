(define  (domain Blocksworld)
  (:requirements :typing)
  (:types zero green_cube_p0201)
  (:predicates
    (zero_fsm0_state0 ?v0 - green_cube_p0201)
    (zero_fsm0_state1)
    (green_cube_p0201_fsm0_state0)
    (green_cube_p0201_fsm0_state1)
    (green_cube_p0201_fsm0_state2)
    (green_cube_p0201_fsm0_state3 ?v0 - zero)
    (green_cube_p0201_fsm0_state4 ?v1 - zero)
  )
  (:action  pick   :parameters  (?zero - zero ?white_cube_p0101 - green_cube_p0201 )
   :precondition   (and
        (zero_fsm0_state1)
   )
   :effect   (and
        (zero_fsm0_state0 ?v0 - green_cube_p0201)
  ))

  (:action  place   :parameters  (?zero - zero ?white_cube_p0205 - green_cube_p0201 )
   :precondition   (and
        (zero_fsm0_state0 ?v0 - green_cube_p0201)
   )
   :effect   (and
        (zero_fsm0_state1)
  ))

  (:action  stack   :parameters  (?zero - zero ?red_cube_p0501 - green_cube_p0201 ?white_cube_p0205 - green_cube_p0201 )
   :precondition   (and
   )
   :effect   (and
  ))

)
