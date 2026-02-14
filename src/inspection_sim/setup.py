from setuptools import find_packages, setup

package_name = "inspection_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "CLAUDE.md"]),
        ("share/" + package_name + "/launch", ["launch/sim_system.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="No-hardware simulation harness (fake drivers) for inspection-robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_agv = inspection_sim.nodes.fake_agv_node:main",
            "fake_arm = inspection_sim.nodes.fake_arm_node:main",
            "fake_perception = inspection_sim.nodes.fake_perception_node:main",
            "fake_planning = inspection_sim.nodes.fake_planning_node:main",
            "fake_defect = inspection_sim.nodes.fake_defect_node:main",
        ],
    },
)
