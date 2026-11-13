from glob import glob

from setuptools import find_packages, setup

package_name = "autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed",
    maintainer_email="mohamed.jabeur2004@gmail.com",
    description="Kalman fusion of wheel odometry + camera lane + lidar, and lane-keeping command.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["fusion_node = autonomy.fusion_node:main",
                                      "speed_display = autonomy.speed_display:main"]},
)
