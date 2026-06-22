import os
from glob import glob

from setuptools import find_packages, setup

package_name = "amr_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    test_suite="test",
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="bong7233",
    maintainer_email="bong7233@example.com",
    description="OpenCV ArUco docking-marker perception for the ROS2_Prac AMR stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aruco_docking_node = amr_vision.aruco_docking_node:main",
            "mock_dock_camera_node = amr_vision.mock_dock_camera_node:main",
        ],
    },
)
