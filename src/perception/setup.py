from glob import glob

from setuptools import find_packages, setup

package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmed",
    maintainer_email="ahmedsalem1and1@gmail.com",
    description="Lidar obstacle detection with lane assignment for the ENIMIA vehicle.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_obstacle_detector = perception.lidar_obstacle_detector:main",
            "line_detector = perception.line_detector:main",
            "camera_fov_marker = perception.camera_fov_marker:main",
        ],
    },
)
