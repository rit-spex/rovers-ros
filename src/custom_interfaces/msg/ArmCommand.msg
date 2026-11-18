# One high-level arm command.
# Published by arm_master_node on /ARM/PLANNER/COMMAND (motion commands only,
# so the planner can validate them) and on /ARM/END_EFFECTOR/COMMAND (only the
# gripper/solenoid commands the end effector executes itself).  Gripper and
# solenoid commands are never forwarded to the planner.
string command
