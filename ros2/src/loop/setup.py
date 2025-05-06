from setuptools import find_packages, setup

package_name = "loop"

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
    maintainer="Colin de Seroux",
    maintainer_email="contact@colindeseroux",
    description="Simple Python Node Loop",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "loop_node = loop.loop_node:main"
        ],
    },
)
