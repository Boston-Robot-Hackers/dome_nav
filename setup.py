from setuptools import find_packages, setup
from glob import glob
import os

package_name = "dome_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pitosalas",
    maintainer_email="pitosalas@gmail.com",
    description="Navigation and SLAM management package for the DOME robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "slam_manager_node = dome_nav.slam_manager_node:main",
            "nav_manager_node = dome_nav.nav_manager_node:main",
            "explorer_manager_node = dome_nav.explorer_manager_node:main",
        ],
    },
)
