from glob import glob

from setuptools import setup

package_name = "ninsar_bag_fix"

setup(
    name=package_name,
    version="0.1.0",
    # the importable module keeps its short name so `python3 -m bagfix ...`
    # and every documented command work unchanged
    packages=["bagfix", "bagfix.transforms"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Luis",
    maintainer_email="luisfelipewb@gmail.com",
    description="Config-driven repair of rosbags recorded on the NINSAR robots.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # Installed into install/ninsar_bag_fix/bin, which colcon puts on PATH, so
    # `bagfix ...` just works after sourcing. This is a plain CLI rather than a
    # node, so it deliberately does not go to lib/<pkg> for `ros2 run`.
    entry_points={
        "console_scripts": ["bagfix = bagfix.cli:main"],
    },
)
