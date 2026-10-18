from glob import glob

from setuptools import setup


package_name = "robotx_mission_state_machine"


setup(
    name=package_name,
    version="0.1.0",
    py_modules=[
        "mission_planner_node",
        "mission_contract",
        "task4_guard",
    ],
    package_dir={
        "": "src",
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotX Team",
    maintainer_email="robotx@example.invalid",
    description="RobotX main mission state machine.",
    license="Apache-2.0",
    #可执行程序名；这里左边的名字，是为了方便执行“ros2 run robotx_mission_state_machine mission_planner”这样子能直接启动--（接第二行）
    # 节点的命令，不一定需要和对应node代码里的super().__init__("mission_planner")里的名字对应”
    entry_points={
        "console_scripts": [
            "mission_planner = mission_planner_node:main",
        ],
    },
)
