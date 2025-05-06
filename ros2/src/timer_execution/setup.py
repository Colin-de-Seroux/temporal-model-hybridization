from setuptools import find_packages, setup

package_name = "timer_execution"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=[]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anna DI PLACIDO",
    maintainer_email="anna@gmail.com",
    description="Timer execution for ROS2 nodes",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "my_node = timer_execution.my_node:main"
        ],
    },
)