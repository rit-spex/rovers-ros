from setuptools import find_packages, setup

package_name = "basestation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/" + package_name + "_launch.xml"]),
        ("lib/" + package_name, [package_name + "/command_codes.py"]),
        ("lib/" + package_name, [package_name + "/encoding.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["basestation_node = basestation.basestation_node:main"],
    },
)
