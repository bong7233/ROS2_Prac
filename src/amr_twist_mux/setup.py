import os
from glob import glob

from setuptools import find_packages, setup

package_name = "amr_twist_mux"

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
    description="Priority multiplexer for /cmd_vel sources in the ROS2_Prac AMR stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "twist_mux_node = amr_twist_mux.twist_mux_node:main",
        ],
    },
)
