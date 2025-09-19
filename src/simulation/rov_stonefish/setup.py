from setuptools import find_packages, setup
from glob import glob
import os

package_name = "rov_stonefish"

data_files_list = []
for f in glob(os.path.join("data", "**", "*"), recursive=True):
    if os.path.isfile(f):  # Only copy files, not directories
        relative_path = os.path.relpath(f, "data")
        target_dir = os.path.join("share", package_name, "data")
        if os.path.dirname(relative_path):  # If inside a subdirectory, preserve it
            target_dir = os.path.join(target_dir, os.path.dirname(relative_path))
        data_files_list.append((target_dir, [f]))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py*'))),
        (os.path.join('share', package_name, 'scenarios'), glob(os.path.join('scenarios', '*.scn*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz*'))),
        (os.path.join('share', package_name, 'nodes'), glob(os.path.join('nodes', '*node.py*'))),
        *data_files_list
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Piotr Flak",
    maintainer_email="piotrflak@student.agh.edu.pl",
    description="ROV stonefish simulation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dvl_pose_translation = nodes.dvl_pose_translation_node:main"
        ],
    },
)
