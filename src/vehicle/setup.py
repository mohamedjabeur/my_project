from glob import glob

from setuptools import find_packages, setup

package_name = "vehicle"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmed",
    maintainer_email="ahmedsalem1and1@gmail.com",
    description="Drive chain (ESP32 + BLDC) of the ENIMIA car.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ackermann_controller = vehicle.ackermann_controller:main",
            "esp32_bridge = vehicle.esp32_bridge:main",
            "virtual_esp32 = vehicle.virtual_esp32:main",
            "wheel_odometry = vehicle.wheel_odometry:main",
        ],
    },
)
