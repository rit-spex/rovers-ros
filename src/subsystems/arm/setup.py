import os

from setuptools import find_packages, setup

package_name = 'arm'
packages = find_packages(where='src', exclude=['test'])
# Map each package to its src-layout directory.  ``arm_constants`` is a
# top-level module at the package root, not under ``src/``: colcon's
# --symlink-install resolves py_modules from the package root only.
package_dir = {
    package: os.path.join('src', *package.split('.')) for package in packages
}

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    package_dir=package_dir,
    # Arm-wide constants are a top-level module at the package root so
    # colcon's --symlink-install can link it into the build space.
    py_modules=['arm_constants'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/' + package_name + '_launch.xml'])
    ],
    install_requires=['setuptools', 'numpy', 'sympy'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Arm control nodes for the SPEX rover.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Canonical names. Launch files and scripts use these.
            'arm_camera_node = arm_camera.arm_camera_node:main',
            'arm_master_node = arm_master.arm_master_node:main',
            'arm_end_effector_node = arm_end_effector.arm_end_effector_node:main',
            'arm_route_planner_node = arm_route_planner.arm_route_planner_node:main',
            'arm_hardware_node = arm_hardware.arm_hardware_node:main',
            'arm_simulated_teensy_node = arm_hardware.arm_simulated_teensy_node:main',
            'arm_teleop_node = arm_teleop.arm_teleop_node:main',
            'arm_typing_node = arm_typing.arm_typing_node:main',
            'arm_sniffer_node = arm_sniffer.arm_sniffer_node:main',
            # Short aliases: `ros2 run arm <name>` without "arm_"/"_node".
            'camera = arm_camera.arm_camera_node:main',
            'master = arm_master.arm_master_node:main',
            'end_effector = arm_end_effector.arm_end_effector_node:main',
            'route_planner = arm_route_planner.arm_route_planner_node:main',
            'hardware = arm_hardware.arm_hardware_node:main',
            'sim_teensy = arm_hardware.arm_simulated_teensy_node:main',
            'teleop = arm_teleop.arm_teleop_node:main',
            'typing = arm_typing.arm_typing_node:main',
            'sniffer = arm_sniffer.arm_sniffer_node:main',
            # Mnemonic aliases for the debug/simulation-only helpers.
            'ASN = arm_sniffer.arm_sniffer_node:main',
            'ASTN = arm_hardware.arm_simulated_teensy_node:main',
        ],
    },
)
