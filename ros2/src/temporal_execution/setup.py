from setuptools import find_packages, setup

package_name = "temporal_execution"

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
    maintainer="Anna DI PLACIDO, Colin de SEROUX",
    maintainer_email="annadiplacido3@gmail.com, contact@colindeseroux.fr",
    description="Temporal execution for ROS2 nodes",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "loop_node = temporal_execution.loop_node:main"
        ],
    },
)