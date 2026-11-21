from glob import glob

from setuptools import find_packages, setup

package_name = "tello_flight"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/plans", glob("plans/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mo",
    maintainer_email="mo@tello-twin.local",
    description="flight_runner and safety_console.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["flight_runner = tello_flight.flight_runner:main",
                                      "safety_console = tello_flight.safety_console:main"]},
)
