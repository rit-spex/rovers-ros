"""Script to control a version of the arm in a 3D pygame window

Controls
- No Buttons (Position Drive)
    - Tx : Translation of wrist along x-axis
    - Ty : Translation of wrist along y-axis
    - Tz : Translation of wrist along z-axis
    - Rx : Rotation of the gripper along the wrist axis
    - Rz : Rotation of the gripper along the lead-screw axis
- Left Button
    - Homing Function : Drives the arm to point straight up
- Right Button (Joint Drive)
    - Rx : Rotation of the elbow
    - Ry : Rotation of the shoulder
    - Rz : Rotation of the base
"""

# This will install stuff
# pip install sympy matplotlib numpy

import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import pygame


from inverse_kinematics.inverse_kinematics.space_mouse import (
    setup_spacemouse,
    read_spacemouse,
    flush_hid,
    flush_hid_preserve_buttons,
)
from inverse_kinematics.inverse_kinematics.math_helpers import (
    calc_angle_between_points,
    get_flipped_angles,
    wrap_to_pi,
    angle_diff,
    Tz,
)
from inverse_kinematics.inverse_kinematics.linear_algebra import (
    inverse_kinematics,
    calc_joint_positions,
    homingStep,
    joint_limits,
    collision_protection,
)
from inverse_kinematics.inverse_kinematics.pygame_graphics import (
    compute_camera_basis,
    project_3d,
    draw_arm,
    draw_link,
    draw_cube,
    draw_chassis,
    draw_GUI,
    init_pygame,
)


# ================== MAIN ==================


# TODO: Animate th4
# TODO: Seperate parameters into that file

# ========== Parameters ==========

# Arm initial parameters
initialAngles = [math.pi, math.pi / 2, -math.pi / 2, 0, 0]
iAs = initialAngles
j0, j1, j2, j3 = calc_joint_positions(iAs[0], iAs[1], iAs[2], iAs[3], False)
th0, th1, th2, th3, th4 = initialAngles


# General parameters
point = j3
running = True
trans_sens = 0.004
rotate_sens = 0.00005
fps = 30

# Colors
colRed = (60, 50, 50)
colGreen = (50, 60, 50)
colBlue = (50, 50, 60)


# ========== Initializations ==========

# Space mouse
device = setup_spacemouse()
state = {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "buttons": 0}
buttonz = [False, False]
homing = False

# Time tracking
startTime = time.time()

# Draw initial window
screen, clock, font, static_surface = init_pygame()
screen.fill(colGreen)
screen.blit(static_surface, (0, 0))
draw_arm(screen, j0, j1, j2, j3)
draw_GUI(screen, font, point, th0, th1, th2, th3, th4, homing, buttonz, static_surface)
pygame.display.flip()


# ========== Main loop ==========


while running:

    # Update pygame
    clock.tick(fps)
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

    # Read mouse data
    read_spacemouse(device, state)
    if int(state["buttons"]) == 1:
        homing = True
    if int(state["buttons"]) == 2:
        buttonz[1] = not (buttonz[1])

    # Check if anything is going on
    if (not all(v == 0 for v in state.values())) or homing:
        # Okay well what is going on
        if homing:
            homing, th0, th1, th2, th3, th4 = homingStep(
                homing, th0, th1, th2, th3, th4
            )
            if homing:
                screen.fill(colBlue)
            else:
                screen.fill(colGreen)
        elif buttonz[1]:
            screen.fill(colGreen)
            th2 = wrap_to_pi(th2 + state["rx"] * rotate_sens)
            _, _, _, point = calc_joint_positions(th0, th1, th2, th3, False)

            th1 = wrap_to_pi(th1 + state["ry"] * rotate_sens)
            _, _, _, point = calc_joint_positions(th0, th1, th2, th3, False)

            th0 = wrap_to_pi(th0 - state["rz"] * rotate_sens)
            _, _, _, point = calc_joint_positions(th0, th1, th2, th3, False)
        else:
            # Move point inside cube
            point[0] += state["x"] * trans_sens
            point[1] += state["y"] * trans_sens
            point[2] += state["z"] * trans_sens

            # x rotates the gripper
            th3 += state["rx"] * rotate_sens
            # z spins the gripper
            th4 += state["rz"] * rotate_sens

            # Compute arm joints
            try:
                th0, th1, th2, _ = inverse_kinematics(point, [th0, th1, th2, th3], th4)

                screen.fill(colGreen)
            except Exception as e:
                print(e)
                screen.fill(colRed)

        # Reset state variable
        for key in state:
            state[key] = 0

        # Update stuff
        if buttonz[1]:
            th0, th1, th2, th3, th4 = joint_limits(th0, th1, th2, th3, th4, False)
        else:
            th0, th1, th2, th3, th4 = joint_limits(th0, th1, th2, th3, th4, True)

        th0, th1, th2, th3, th4 = collision_protection(th0, th1, th2, th3, th4)
        l0, l1, l2, l3 = calc_joint_positions(th0, th1, th2, th3)
        point = l2

        # Draw
        draw_GUI(
            screen,
            font,
            point,
            th0,
            th1,
            th2,
            th3,
            th4,
            homing,
            buttonz,
            static_surface,
        )
        print(
            f"{time.time()-startTime:10.4f}\t\t{th0*57.3:10.4f}\t\t{th1*57.3:10.4f}\t\t{th2*57.3:10.4f}\t\t{th3*57.3:10.4f}"
        )
        pygame.display.flip()


# Cleanup
pygame.quit()
device.close()
