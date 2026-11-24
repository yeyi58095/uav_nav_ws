from setuptools import setup


setup(
    name="uav_nav",
    version="0.1.0",
    packages=["uav_nav"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/uav_nav"]),
        ("share/uav_nav", ["package.xml"]),
        ("share/uav_nav/config", ["config/avoidance.parm"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "sim_lidar = uav_nav.lidar:main",
            "sitl_bridge = uav_nav.bridge:main",
        ],
    },
)
