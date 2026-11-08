from setuptools import find_packages, setup

package_name = 'auto_led'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blueboi',
    maintainer_email='az3828@rit.edu',
    description='TODO: AUTO LED',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_led_node = auto_led.auto_led_node:main'
        ],
    },
)
