import os

from setuptools import find_packages, setup

package_name = 'arm'
packages = find_packages(where='src', exclude=['test'])
# Map each package to its src-layout directory.  Do not map the root package:
# `constants` must resolve to the package-root symlink so colcon's
# --symlink-install can link it into the build space.
package_dir = {
    package: os.path.join('src', *package.split('.')) for package in packages
}

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    package_dir=package_dir,
    # Arm-wide constants live in src/ and are exposed at the package root.
    py_modules=['constants'],
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
            'arm_camera_node = arm_camera.arm_camera_node:main',
            'arm_controller_node = arm_controller.arm_controller_node:main',
            'arm_hardware_node = arm_hardware.arm_hardware_node:main',
            'arm_teleop_node = arm_teleop.arm_teleop_node:main',
            'arm_sniffer_node = arm_sniffer.arm_sniffer_node:main',
        ],
    },
)
