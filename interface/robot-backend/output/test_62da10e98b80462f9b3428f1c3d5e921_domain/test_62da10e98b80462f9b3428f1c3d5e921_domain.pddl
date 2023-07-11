(define  (domain test_62da10e98b80462f9b3428f1c3d5e921_domain)
  (:requirements :typing)
  (:types zero blue_cube_4_p0006:1)
  (:predicates
    (zero_fsm0_state0)
    (zero_fsm0_state1)
    (blue_cube_4_p0006:1_fsm0_state0)
    (blue_cube_4_p0006:1_fsm0_state1 ?v0 - zero)
    (blue_cube_4_p0006:1_fsm0_state2)
  )
  (:action  place   :parameters  (?zero - zero ?red_brick_1_p0208:p0209:1:false - blue_cube_4_p0006:1 )
   :precondition   (and
        (zero_fsm0_state0)
   )
   :effect   (and
        (zero_fsm0_state1)
  ))

  (:action  pick   :parameters  (?zero - zero ?red_brick_1_p0000:p0001:1:false - blue_cube_4_p0006:1 )
   :precondition   (and
        (zero_fsm0_state1)
   )
   :effect   (and
        (zero_fsm0_state0)
  ))

)
