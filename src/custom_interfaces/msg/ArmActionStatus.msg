# /ARM/ACTION/STATUS — arm_action_driver_node state and its latest event.
# active mirrors /ARM/ACTION/ACTIVE; phase details the substate.
# error_seq increments for every new rejection/clear event (like
# ArmPlannerStatus) so the master can republish /ARM/STATE/ERROR without
# re-deriving it.
bool ready            # all five measured joints have reported in
uint8 active          # 0=IDLE 1=HOMING 2=FLIPPING 3=ZENITH 4=RESTING
uint8 phase           # action substate (PHASE_* constants in arm_constants)
uint8 error_code      # 0 = none, else a rover_protocol.arm_errors value
uint8 error_arg       # joint index or other context for error_code
uint8 error_seq       # increments for every new error event, incl. clear
uint8 homing_mask     # bitmask of joints the active homing action drives
float32 progress      # 0.0..1.0 within the current move
