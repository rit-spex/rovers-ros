# Keyboard face plane in the rover base frame (+X right, +Y forward, +Z up).
# The origin is the keyboard centre.  +X runs along the keyboard's horizontal
# axis, +Y runs up the face, and +Z points out of the keyboard toward the
# rover.  Positions are metres; angles are radians (roll, pitch, yaw, Rz*Ry*Rx).
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw

# Which side of the rover the keyboard is on:
#   +1 keyboard is on the left (arm base near +90 deg)
#   -1 keyboard is on the right (arm base near -90 deg)
#    0 infer the side from the sign of x
int8 side
