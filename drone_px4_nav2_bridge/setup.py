from setuptools import find_packages, setup


package_name = "drone_px4_nav2_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Evan",
    maintainer_email="yuanyifan0625@gmail.com",
    description="PX4-to-ROS bridge package for MAV1 Nav2 integration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
