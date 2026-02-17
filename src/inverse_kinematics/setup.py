from setuptools import find_packages, setup

package_name = 'inverse_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/' + package_name + '_launch.xml']),
        ("lib/" + package_name, [package_name + "/armParameters.py"]),
        ("lib/" + package_name, [package_name + "/linearAlgebra.py"]),
        ("lib/" + package_name, [package_name + "/mathHelpers.py"]),
        ("lib/" + package_name, [package_name + "/pygameGraphics.py"]),
        ("lib/" + package_name, [package_name + "/spaceMouse.py"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='garrett',
    maintainer_email='garrett59100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'inverse_kinematics_node = inverse_kinematics.inverse_kinematics_node:main'
        ],
    },
)
