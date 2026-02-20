import math
import time
import pygame
from linear_algebra import (
    calc_joint_positions,
    collision_protection,
    homingStep,
    inverse_kinematics,
    joint_limits,
)
from math_helpers import wrap_to_pi
from pygame_graphics import (
    draw_arm,
    draw_GUI,
    init_pygame,
)
from space_mouse import (
    read_spacemouse,
    setup_spacemouse,
)

# Typing
from rclpy.node import Node
from typing import Any
from pygame import Surface
from pygame.time import Clock
from pygame.font import Font
from numpy import dtype, ndarray


"""
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


class ArmController(Node):
    # Arm initial parameters
    __initial_angles: list[float]
    __iAs: list[float]
    __j0: ndarray[tuple[int], dtype[Any]]
    __j1: ndarray[tuple[int], dtype[Any]]
    __j2: ndarray[tuple[int], dtype[Any]]
    __j3: ndarray[tuple[int], dtype[Any]]
    __th0: float
    __th1: float
    __th2: float
    __th3: float
    __th4: float

    # General parameters
    __point: ndarray[tuple[int], dtype[Any]]
    __running: bool
    __trans_sens: float
    __rotate_sens: float
    __fps: int

    # Colors
    __col_red: tuple[int, int, int]
    __col_green: tuple[int, int, int]
    __col_blue: tuple[int, int, int]

    # Space mouse
    __device: Any
    __state: dict[str, int]
    __buttonz: list[bool]
    __homing: bool

    # Time tracking
    __start_time: float

    # Pygame
    __screen: Surface
    __clock: Clock
    __font: Font
    __static_surface: Surface

    def __init__(self):
        pass
        # Arm initial parameters
        self.__initial_angles = [math.pi, math.pi / 2, -math.pi / 2, 0.0, 0.0]
        self.__iAs = self.__initial_angles
        self.__j0, self.__j1, self.__j2, self.__j3 = calc_joint_positions(
            self.__iAs[0], self.__iAs[1], self.__iAs[2], self.__iAs[3], False
        )
        self.__th0, self.__th1, self.__th2, self.__th3, __th4 = self.__initial_angles

        # General parameters
        self.__point = self.__j3
        self.__running = True
        self.__trans_sens = 0.004
        self.__rotate_sens = 0.00005
        self.__fps = 30

        # Colors
        self.__col_red = (60, 50, 50)
        self.__col_green = (50, 60, 50)
        self.__col_blue = (50, 50, 60)

        # Space mouse
        self.__device = setup_spacemouse()
        print(self.__device)
        self.__state = {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "buttons": 0}
        self.__buttonz = [False, False]
        self.__homing = False

        # Time tracking
        self.__start_time = time.time()

        # Initialize Pygame
        self.__screen, self.__clock, self.__font, self.__static_surface = init_pygame()

    def run(self):
        self.__screen.fill(self.__col_green)
        self.__screen.blit(self.__static_surface, (0, 0))
        draw_arm(self.__screen, self.__j0, self.__j1, self.__j2, self.__j3)
        draw_GUI(
            self.__screen,
            self.__font,
            self.__point,
            self.__th0,
            self.__th1,
            self.__th2,
            self.__th3,
            self.__th4,
            self.__homing,
            self.__buttonz,
            self.__static_surface,
        )
        pygame.display.flip()

        while self.__running:
            # Update pygame
            self.__clock.tick(self.__fps)
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    self.__running = False

            # Read mouse data
            read_spacemouse(self.__device, self.__state)
            if int(self.__state["buttons"]) == 1:
                self.__homing = True
            if int(self.__state["buttons"]) == 2:
                self.__buttonz[1] = not (self.__buttonz[1])

            # Check if anything is going on
            if (not all(v == 0 for v in self.__state.values())) or self.__homing:
                # Okay well what is going on
                if self.__homing:
                    (
                        self.__homing,
                        self.__th0,
                        self.__th1,
                        self.__th2,
                        self.__th3,
                        self.__th4,
                    ) = homingStep(
                        self.__homing,
                        self.__th0,
                        self.__th1,
                        self.__th2,
                        self.__th3,
                        self.__th4,
                    )
                    if self.__homing:
                        self.__screen.fill(self.__col_blue)
                    else:
                        self.__screen.fill(self.__col_green)
                elif self.__buttonz[1]:
                    self.__screen.fill(self.__col_green)
                    self.__th2 = wrap_to_pi(
                        self.__th2 + self.__state["rx"] * self.__rotate_sens
                    )
                    _, _, _, self.__point = calc_joint_positions(
                        self.__th0, self.__th1, self.__th2, self.__th3, False
                    )

                    self.__th1 = wrap_to_pi(
                        self.__th1 + self.__state["ry"] * self.__rotate_sens
                    )
                    _, _, _, self.__point = calc_joint_positions(
                        self.__th0, self.__th1, self.__th2, self.__th3, False
                    )

                    self.__th0 = wrap_to_pi(
                        self.__th0 - self.__state["rz"] * self.__rotate_sens
                    )
                    _, _, _, self.__point = calc_joint_positions(
                        self.__th0, self.__th1, self.__th2, self.__th3, False
                    )
                else:
                    # Move point inside cube
                    self.__point[0] += self.__state["x"] * self.__trans_sens
                    self.__point[1] += self.__state["y"] * self.__trans_sens
                    self.__point[2] += self.__state["z"] * self.__trans_sens

                    # x rotates the gripper
                    self.__th3 += self.__state["rx"] * self.__rotate_sens
                    # z spins the gripper
                    self.__th4 += self.__state["rz"] * self.__rotate_sens

                    # Compute arm joints
                    try:
                        self.__th0, self.__th1, self.__th2, _ = inverse_kinematics(
                            self.__point,
                            [self.__th0, self.__th1, self.__th2, self.__th3],
                            self.__th4,
                        )

                        self.__screen.fill(self.__col_green)
                    except Exception as e:
                        print(e)
                        self.__screen.fill(self.__col_red)

                # Reset state variable
                for key in self.__state:
                    self.__state[key] = 0

                # Update stuff
                if self.__buttonz[1]:
                    self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = (
                        joint_limits(
                            self.__th0,
                            self.__th1,
                            self.__th2,
                            self.__th3,
                            self.__th4,
                            False,
                        )
                    )
                else:
                    self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = (
                        joint_limits(
                            self.__th0,
                            self.__th1,
                            self.__th2,
                            self.__th3,
                            self.__th4,
                            True,
                        )
                    )

                self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = (
                    collision_protection(
                        self.__th0, self.__th1, self.__th2, self.__th3, self.__th4
                    )
                )
                _, _, l2, _ = calc_joint_positions(
                    self.__th0, self.__th1, self.__th2, self.__th3
                )
                self.__point = l2

                # Draw
                draw_GUI(
                    self.__screen,
                    self.__font,
                    self.__point,
                    self.__th0,
                    self.__th1,
                    self.__th2,
                    self.__th3,
                    self.__th4,
                    self.__homing,
                    self.__buttonz,
                    self.__static_surface,
                )
                print(
                    f"{time.time()-self.__start_time:10.4f}\t\t{self.__th0*57.3:10.4f}\t\t{self.__th1*57.3:10.4f}\t\t{self.__th2*57.3:10.4f}\t\t{self.__th3*57.3:10.4f}"
                )
                pygame.display.flip()

        pygame.quit()
        self.__device.close()


def main():
    arm_controller = ArmController()
    arm_controller.run()

if __name__ == "__main__":
    main()
