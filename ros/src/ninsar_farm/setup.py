from glob import glob

from setuptools import find_packages, setup

package_name = "ninsar_farm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    # the farm editor's web UI is served from the package itself
    package_data={package_name: ["editor/static/**/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="Luis Felipe Wolf Batista",
    maintainer_email="luisfelipewb@gmail.com",
    description="NINSAR farm description tools: farm GeoJSON library and editor, farm scene graph builders.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "farm_geojson = ninsar_farm.farm_geojson:main",
            "farm_editor = ninsar_farm.editor.serve:main",
            "geojson_to_dsg = ninsar_farm.geojson_to_dsg:main",
            "farm_dsg = ninsar_farm.farm_dsg:main",
            "make_farm_dsg = ninsar_farm.make_farm_dsg:main",
            "make_isaac_farm_dsg = ninsar_farm.make_isaac_farm_dsg:main",
        ],
    },
)
