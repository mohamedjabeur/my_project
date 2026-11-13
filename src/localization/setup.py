from glob import glob

from setuptools import find_packages, setup

package_name = "localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmed",
    maintainer_email="ahmedsalem1and1@gmail.com",
    description="Gazebo spawn, teleop drive path and ground-truth odometry TF for the ENIMIA vehicle.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_to_drive = localization.cmd_vel_to_drive:main",
            "odom_tf = localization.odom_tf:main",
        ],
    },
)
