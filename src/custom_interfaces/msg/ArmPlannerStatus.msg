# Route planner readiness and its latest error event.
# error_seq increments for every new event, including a clear (code 0), so
# the controller can republish /ARM/STATE/ERROR without re-deriving it.
bool ready
uint8 error_seq
uint8 error_code
uint8 error_arg
