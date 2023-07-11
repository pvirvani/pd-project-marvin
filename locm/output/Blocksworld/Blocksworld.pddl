(define  (domain Blocksworld)
  (:requirements :typing)
  (:types zero b2)
  (:predicates
    (zero_fsm0_state0 ?v0 - b2)
    (zero_fsm0_state1 ?v3 - b2 ?v5 - b2)
    (b2_fsm0_state0 ?v1 - zero)
    (b2_fsm0_state1)
    (b2_fsm0_state2)
    (b2_fsm0_state3 ?v0 - zero)
    (b2_fsm1_state0)
    (b2_fsm1_state1 ?v1 - zero)
    (b2_fsm1_state2 ?v0 - zero ?v2 - b2)
    (b2_fsm2_state0 ?v5 - zero ?v13 - b2)
    (b2_fsm2_state1 ?v1 - b2 ?v2 - zero)
    (b2_fsm2_state2 ?v0 - zero)
  )
  (:action  stack   :parameters  (?zero - zero ?b1 - b2 ?b3 - b2 )
   :precondition   (and
        (zero_fsm0_state0 ?v0 - b2)
        (b2_fsm0_state0 ?v1 - zero)
        (b2_fsm0_state3 ?v0 - zero)
        (b2_fsm1_state1 ?v1 - zero)
        (b2_fsm2_state1 ?v1 - b2 ?v2 - zero)
        (b2_fsm2_state2 ?v0 - zero)
   )
   :effect   (and
        (zero_fsm0_state1 ?v3 - b2 ?v5 - b2)
        (b2_fsm0_state1)
        (b2_fsm0_state0 ?v1 - zero)
        (b2_fsm1_state2 ?v0 - zero ?v2 - b2)
        (b2_fsm2_state0 ?v5 - zero ?v13 - b2)
        (b2_fsm2_state1 ?v1 - b2 ?v2 - zero)
  ))

  (:action  unstack   :parameters  (?zero - zero ?b2 - b2 ?b1 - b2 )
   :precondition   (and
        (b2_fsm1_state2 ?v0 - zero ?v2 - b2)
        (b2_fsm2_state0 ?v5 - zero ?v13 - b2)
        (b2_fsm2_state1 ?v1 - b2 ?v2 - zero)
   )
   :effect   (and
        (b2_fsm1_state1 ?v1 - zero)
        (b2_fsm2_state1 ?v1 - b2 ?v2 - zero)
        (b2_fsm2_state2 ?v0 - zero)
  ))

  (:action  pick   :parameters  (?zero - zero ?b1 - b2 )
   :precondition   (and
        (zero_fsm0_state1 ?v3 - b2 ?v5 - b2)
        (b2_fsm0_state2)
   )
   :effect   (and
        (zero_fsm0_state0 ?v0 - b2)
        (b2_fsm0_state3 ?v0 - zero)
  ))

  (:action  putdown   :parameters  (?zero - zero ?b2 - b2 )
   :precondition   (and
        (b2_fsm1_state1 ?v1 - zero)
   )
   :effect   (and
        (b2_fsm1_state0)
  ))

)
