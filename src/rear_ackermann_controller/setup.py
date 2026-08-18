"""실행 가능한 노드 진입점 등록"""

from setuptools import find_packages, setup

package_name = "rear_ackermann_controller"
workspace_params = "../params_setting.json"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [workspace_params]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Simple rear-wheel steering controller",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rear_ackermann_node=rear_ackermann_controller.rear_ackermann_node:main",
        ],
    },
)
