from glob import glob

from setuptools import find_packages, setup

package_name = "robofetch_factory"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/scenarios", glob("config/scenarios/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ArianShafagh",
    maintainer_email="arian.shafagh2003@gmail.com",
    description="Factory layout, points of interest, path lengths and live production sections.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "poi = robofetch_factory.layout:poi_cli",
            "factory_node = robofetch_factory.factory_node:main",
            "factory_monitor = robofetch_factory.monitor:main",
        ],
    },
)
