from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/last.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spex-rover',
    maintainer_email='cullen.straub@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'object_detection_node = object_detection.object_finder_control:main',
        'object_tracker_node = object_detection.object_tracker:main',
        'arcu_tracker_node = object_detection.arcu_tracker:main',
        'aruco_pathfinding_node = object_detection.aruco_pathfinding:main'
        ], 
    },
)
