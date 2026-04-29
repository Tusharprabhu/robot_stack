from setuptools import find_packages, setup

package_name = "rplidar_2d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="ROS 2 Humble RPLidar 2D publisher using the Python rplidar library.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rplidar_node = rplidar_2d.rplidar_node:main",
        ]
    },
)
