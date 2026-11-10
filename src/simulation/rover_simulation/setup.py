from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py') + glob('launch/*.xml')),
        ('share/' + package_name + '/description',
            glob('description/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Garrett Tupper',
    maintainer_email='garrett59100@gmail.com',
    description='Gazebo assets and simulated hardware bridges for the rover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_bridge = rover_simulation.gazebo_bridge:main',
            'sim_config_node = rover_simulation.sim_config_node:main',
        ],
    },
)
