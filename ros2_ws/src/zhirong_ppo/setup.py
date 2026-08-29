from glob import glob

from setuptools import find_packages, setup


package_name = "zhirong_ppo"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhirong Team",
    maintainer_email="maintainer@example.com",
    description="PPO local-avoidance experiment for the Zhirong robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ppo_smoke = zhirong_ppo.smoke:main",
            "ppo_train = zhirong_ppo.train:main",
            "ppo_evaluate = zhirong_ppo.evaluate:main",
        ],
    },
)
