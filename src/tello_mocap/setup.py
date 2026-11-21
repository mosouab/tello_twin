from setuptools import find_packages, setup

package_name = "tello_mocap"

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
    maintainer="mo",
    maintainer_email="mo@tello-twin.local",
    description="Vicon input for the Tello stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["mocap_adapter = tello_mocap.mocap_adapter:main",
                                      "mocap_sim = tello_mocap.mocap_sim:main"]},
)
