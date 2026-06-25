from setuptools import find_packages, setup

package_name = "robot_lab_sim_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="thw2188",
    maintainer_email="thw2188@example.com",
    description="Gazebo Classic simulation helper nodes for the robot lab demo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "soft_gripper_attach = robot_lab_sim_tools.soft_gripper_attach:main",
        ],
    },
)
