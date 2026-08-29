from glob import glob
from setuptools import find_packages, setup


package_name = "zhirong_tasks"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhirong Team",
    maintainer_email="maintainer@example.com",
    description="Task queue and navigation state machine for the Zhirong robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_manager = zhirong_tasks.task_manager:main",
            "task_cli = zhirong_tasks.task_cli:main",
        ],
    },
)
