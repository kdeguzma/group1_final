from setuptools import find_packages, setup

package_name = "group1_final"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/search_and_rescue.launch.py"],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/mission_params.yaml",
                "config/nav2_params.yaml",
            ],
        ),
        (
            "share/" + package_name + "/maps",
            [
                "maps/final_project_map.yaml",
                "maps/final_project_map.pgm",
            ],
        ),
        (
            "share/" + package_name + "/rviz",
            ["rviz/nav2.rviz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kdeguzma",
    maintainer_email="kdeguzma@umd.edu",
    description="Final project package for search and rescue",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "report_survivor_server_exe = group1_final.service_servers.report_survivor_server:main",
            "detect_survivor_server_exe = group1_final.service_servers.detect_survivor_server:main",
            "search_and_rescue_exe = group1_final.scripts.main_search_and_rescue:main",
        ],
    },
)
