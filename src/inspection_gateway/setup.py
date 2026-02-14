from setuptools import find_packages, setup

package_name = "inspection_gateway"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "CLAUDE.md"]),
        ("share/" + package_name + "/launch", ["launch/inspection_gateway.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="gRPC gateway bridging inspection-hmi and ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "inspection_gateway = inspection_gateway.main:main",
        ],
    },
)
