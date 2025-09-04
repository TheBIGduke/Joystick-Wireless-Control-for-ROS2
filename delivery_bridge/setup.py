from setuptools import find_packages, setup

package_name = "delivery_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cruz Mauricio Arteaga-Escamilla",
    maintainer_email="cmauricioae8@gmail.com",
    description="This package is a base bridge between ROS2 ecosystem and a Web Application for an autonomous mobile robot",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["server_node = delivery_bridge.server_node:main"],
    },
)
