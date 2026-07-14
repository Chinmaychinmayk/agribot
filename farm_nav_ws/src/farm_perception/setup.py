from setuptools import find_packages, setup

package_name = "farm_perception"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chinmay",
    maintainer_email="chinmaychinmay2003@gmail.com",
    description="Depth-geometric farm-lane perception, plant counting and mission control.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_perception_node = farm_perception.lane_perception_node:main",
            "plant_counter_node = farm_perception.plant_counter_node:main",
            "robot_spawner_node = farm_perception.robot_spawner_node:main",
            "goal_sender_node = farm_perception.goal_sender_node:main",
            "path_recorder_node = farm_perception.path_recorder_node:main",
            "video_recorder_node = farm_perception.video_recorder_node:main",
        ],
    },
)
