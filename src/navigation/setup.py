from glob import glob

from setuptools import find_packages, setup

package_name = "navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmed",
    maintainer_email="ahmedsalem1and1@gmail.com",
    description="Nav2 autonomous navigation for the ENIMIA car.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "send_goal = navigation.send_goal:main",
            "goal_relay = navigation.goal_relay:main",
            "world_markers = navigation.world_markers:main",
        ],
    },
)
