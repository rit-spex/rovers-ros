import pygame
import numpy as np
from linear_algebra import calc_joint_positions


def compute_camera_basis():
    C = np.array(CAM_POS)
    L = np.array(LOOK_AT)
    F = L - C
    F /= np.linalg.norm(F)
    R = np.cross(F, [0, 1, 0])
    R /= np.linalg.norm(R)
    U = np.cross(R, F)
    return C, R, U, F


def project_3d(x, y, z):
    """
    Projects a 3D point to 2D screen coordinates.
    Camera is at CAM_POS, looking at LOOK_AT.
    """
    # Convert to numpy arrays
    P = np.array([x, y, z])
    # C = np.array(CAM_POS)
    # L = np.array(LOOK_AT)

    # # Forward vector from camera to look-at point
    # F = (L - C)
    # F = F / np.linalg.norm(F)

    # # Right vector
    # up = np.array([0, 1, 0])
    # R = np.cross(F, up)
    # R = R / np.linalg.norm(R)

    # # True up vector
    # U = np.cross(R, F)

    # Vector from camera to point
    V = P - C

    # Coordinates in camera space
    x_cam = np.dot(V, R)
    y_cam = np.dot(V, U)
    z_cam = np.dot(V, F)

    # Simple perspective projection
    if z_cam <= 1:
        z_cam = 1  # avoid division by zero

    scale = 300 / z_cam
    sx = WIDTH / 2 + x_cam * scale
    sy = HEIGHT / 2 - y_cam * scale  # y down
    return int(sx), int(sy), scale


def draw_cube(cube_size, screen):
    """
    Draw cube edges, internal gridlines, and main axes from the -corner.
    Gridlines are drawn on all 6 cube faces.
    """
    half = cube_size / 2

    subdivisions = 4

    # Cube corners
    corners = [
        [-half, -half, -half],  # 0
        [half, -half, -half],  # 1
        [half, half, -half],  # 2
        [-half, half, -half],  # 3
        [-half, -half, half],  # 4
        [half, -half, half],  # 5
        [half, half, half],  # 6
        [-half, half, half],  # 7
    ]

    # Cube edges
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]

    # Projected corners
    proj_corners = [project_3d(*c) for c in corners]

    # Draw cube edges
    for e in edges:
        c1, c2 = proj_corners[e[0]], proj_corners[e[1]]
        pygame.draw.line(screen, (200, 200, 200), c1[:2], c2[:2], 1)

    # Step size for internal gridlines
    step = cube_size / subdivisions

    # Draw gridlines on all faces
    for i in range(1, subdivisions):
        offset = -half + i * step

        # XY faces (z = constant)
        for z in [-half, half]:
            for x in range(subdivisions + 1):
                x_pos = -half + x * step
                p1 = project_3d(x_pos, -half, z)
                p2 = project_3d(x_pos, half, z)
                pygame.draw.line(screen, (120, 120, 120), p1[:2], p2[:2], 1)
            for y in range(subdivisions + 1):
                y_pos = -half + y * step
                p1 = project_3d(-half, y_pos, z)
                p2 = project_3d(half, y_pos, z)
                pygame.draw.line(screen, (120, 120, 120), p1[:2], p2[:2], 1)

        # XZ faces (y = constant)
        for y in [-half, half]:
            for x in range(subdivisions + 1):
                x_pos = -half + x * step
                p1 = project_3d(x_pos, y, -half)
                p2 = project_3d(x_pos, y, half)
                pygame.draw.line(screen, (120, 120, 120), p1[:2], p2[:2], 1)
            for z_pos_index in range(subdivisions + 1):
                z_pos = -half + z_pos_index * step
                p1 = project_3d(-half, y, z_pos)
                p2 = project_3d(half, y, z_pos)
                pygame.draw.line(screen, (120, 120, 120), p1[:2], p2[:2], 1)

        # YZ faces (x = constant)
        for x in [-half, half]:
            for y in range(subdivisions + 1):
                y_pos = -half + y * step
                p1 = project_3d(x, y_pos, -half)
                p2 = project_3d(x, y_pos, half)
                pygame.draw.line(screen, (120, 120, 120), p1[:2], p2[:2], 1)
            for z_pos_index in range(subdivisions + 1):
                z_pos = -half + z_pos_index * step
                p1 = project_3d(x, -half, z_pos)
                p2 = project_3d(x, half, z_pos)
                pygame.draw.line(screen, (120, 120, 120), p1[:2], p2[:2], 1)

    # ================== Main axes from -corner ==================
    origin = [-half, -half, -half]

    # X-axis (red)
    x_end = [half, -half, -half]
    px1, py1, _ = project_3d(*origin)
    px2, py2, _ = project_3d(*x_end)
    pygame.draw.line(screen, (255, 0, 0), (px1, py1), (px2, py2), 2)

    # Y-axis (blue)
    y_end = [-half, half, -half]
    py2x, py2y, _ = project_3d(*y_end)
    pygame.draw.line(screen, (0, 126, 255), (px1, py1), (py2x, py2y), 2)

    # Z-axis (green)
    z_end = [-half, -half, half]
    pz2x, pz2y, _ = project_3d(*z_end)
    pygame.draw.line(screen, (0, 255, 0), (px1, py1), (pz2x, pz2y), 2)


def draw_link(screen, p0, p1, color, width=3):
    a = project_3d(*p0)
    b = project_3d(*p1)
    pygame.draw.line(screen, color, a[:2], b[:2], width)
    pygame.draw.circle(screen, color, a[:2], 4)
    pygame.draw.circle(screen, color, b[:2], 4)


def draw_chassis(screen):
    # ---------- Chassis ----------
    x0, y0, z0 = -6.7, -8.5, -6
    x1, y1, z1 = 6.7, 8.5, 0

    corners = [
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1],
    ]

    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]

    proj = [project_3d(*c) for c in corners]
    for i, j in edges:
        pygame.draw.line(screen, (100, 100, 200), proj[i][:2], proj[j][:2], 2)

    pygame.draw.line(
        screen,
        (100, 100, 200),
        project_3d(x0, y0, z0)[:2],
        project_3d(x0, y0, z1 + 36)[:2],
        2,
    )

    pygame.draw.line(
        screen,
        (100, 100, 200),
        project_3d(x1, y0, z0)[:2],
        project_3d(x1, y0, z1 + 24)[:2],
        2,
    )


def draw_arm(screen, l0, l1, l2, l3):
    """
    Draws:
    - 3-link arm (l0 -> l3)
    - Motor offset vectors
    All using project_3d for pygame rendering.
    """

    # ---------- Helper ----------

    # ---------- Arm ----------
    draw_link(screen, l0, l1, (200, 200, 200), 3)
    draw_link(screen, l0 + [0, 0, 2], l1 + [0, 0, 2], (200, 200, 200), 3)
    draw_link(screen, l1, l2, (200, 150, 150), 3)
    draw_link(screen, l1 + [0, 0, 2], l2 + [0, 0, 2], (200, 150, 150), 3)
    draw_link(screen, l2, l3, (255, 80, 80), 3)

    # ---------- Motors ----------
    motor_vec = np.cross(l1, l2)
    norm = np.linalg.norm(motor_vec)
    if norm > 1e-6:
        motor_vec = motor_vec / norm * 5
        m1 = l1 + motor_vec
        m2 = l2 + motor_vec

        for a, b in [(m1, l1), (m2, l2)]:
            p0 = project_3d(*a)
            p1 = project_3d(*b)
            pygame.draw.line(screen, (0, 0, 0), p0[:2], p1[:2], 2)


def draw_GUI(
    screen, font, point, th0, th1, th2, th3, th4, homing, buttonz, static_surface
):
    """
    Draws the robot arm, the point, and GUI info onto the Pygame screen.

    screen: Pygame display surface
    font: Pygame font object
    point: [x, y, z] position of the point
    th0-th3: joint angles in radians
    homing: bool
    buttonz: list of button states
    static_surface: pre-rendered background surface
    """
    l0, l1, l2, l3 = calc_joint_positions(th0, th1, th2, th3)

    # Draw background and arm
    screen.blit(static_surface, (0, 0))
    draw_arm(screen, l0, l1, l2, l3)

    # Draw the point
    proj = project_3d(*point)
    if proj:
        x2d, y2d, s = proj
        r = min(50, max(2, int(6 * s)))
        pygame.draw.circle(screen, (122, 255, 255), (x2d, y2d), r // 2)

    # Display coordinates
    x_text = font.render(f"X: {point[0]:.2f}  ", True, (255, 0, 0))
    y_text = font.render(f"Y: {point[1]:.2f}  ", True, (0, 126, 255))
    z_text = font.render(f"Z: {point[2]:.2f}", True, (0, 255, 0))

    screen.blit(x_text, (10, 10))
    screen.blit(y_text, (10 + x_text.get_width(), 10))
    screen.blit(z_text, (10 + x_text.get_width() + y_text.get_width(), 10))

    # Display angles
    t0_text = font.render(f"T0: {th0*57.3:7.2f}  ", True, (200, 200, 200))
    t1_text = font.render(f"T1: {th1*57.3:7.2f}  ", True, (200, 200, 200))
    t2_text = font.render(f"T2: {th2*57.3:7.2f}  ", True, (200, 200, 200))
    t3_text = font.render(f"T3: {th3*57.3:7.2f}  ", True, (200, 200, 200))
    t4_text = font.render(f"T4: {th4*57.3:7.2f}  ", True, (200, 200, 200))

    y_offset = 10 + x_text.get_height() + 5
    screen.blit(t0_text, (10, y_offset))
    screen.blit(t1_text, (10 + t0_text.get_width(), y_offset))
    screen.blit(t2_text, (10 + t0_text.get_width() + t1_text.get_width(), y_offset))
    screen.blit(
        t3_text,
        (
            10 + t0_text.get_width() + t1_text.get_width() + t2_text.get_width(),
            y_offset,
        ),
    )
    screen.blit(
        t4_text,
        (
            10
            + t0_text.get_width()
            + t1_text.get_width()
            + t2_text.get_width()
            + t3_text.get_width(),
            y_offset,
        ),
    )

    # Display boolean states
    if homing:
        b1_surf = font.render(f"Homing: ACTIVE!", True, (255, 255, 0))
    else:
        b1_surf = font.render(f"Homing: Inactive", True, (255, 255, 255))

    if buttonz[1]:
        b2_surf = font.render(f"Drive Mode: Angle", True, (255, 255, 0))
    else:
        b2_surf = font.render(f"Drive Mode: Position", True, (255, 255, 255))

    screen.blit(b1_surf, (10, 60))
    screen.blit(b2_surf, (10, 60 + b1_surf.get_height() + 5))


def init_pygame():

    # Pygame window
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("3D Cube with Moving Point")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)

    # Initialize static graphical items
    static_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    draw_cube(cube_size, static_surface)
    draw_chassis(static_surface)

    return screen, clock, font, static_surface


# Graphic and camera settings
cube_size = 125  # smaller cube so points are closer to camera
WIDTH = 900
HEIGHT = 700
half = cube_size / 1.2
CAM_POS = [-half * 0.25, -half * 1.25, half * 0.5]  # slightly outside the cube
LOOK_AT = [-cube_size / 4, 0, -10]  # center of the cube
C, R, U, F = compute_camera_basis()
