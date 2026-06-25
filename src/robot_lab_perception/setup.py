from setuptools import find_packages, setup

package_name = "robot_lab_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="thw2188",
    maintainer_email="thw2188@example.com",
    description="RGB-D object detection and 6D pose estimation for the robot lab competition demo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "object_pose_estimator = robot_lab_perception.object_pose_estimator:main",
            "evaluate_perception = robot_lab_perception.evaluate_perception:main",
        ],
    },
)
