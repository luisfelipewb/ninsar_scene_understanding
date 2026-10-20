from glob import glob

from setuptools import find_packages, setup

package_name = "ninsar_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.*")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Luis Felipe Wolf Batista",
    maintainer_email="luisfelipewb@gmail.com",
    description="NINSAR semantic mapping: launch files, configuration and helper nodes around Voxeland and Hydra.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "tf_to_pose_cov = ninsar_mapping.tf_to_pose_cov:main",
            "semantic_cloud_adapter = ninsar_mapping.semantic_cloud_adapter:main",
            "voxel_query = ninsar_mapping.voxel_query:main",
        ],
    },
)
