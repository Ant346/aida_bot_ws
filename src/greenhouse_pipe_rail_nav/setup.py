from setuptools import find_packages, setup


package_name = "greenhouse_pipe_rail_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/pipe_rail_autodock.launch.py",
                "launch/pipe_rail_rviz.launch.py",
            ],
        ),
        (f"share/{package_name}/config", ["config/default.yaml"]),
        (f"share/{package_name}/urdf", ["urdf/aida_placeholder.urdf"]),
        (f"share/{package_name}/rviz", ["rviz/pipe_rail.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIDA",
    maintainer_email="robot@example.com",
    description="Bird-view greenhouse pipe rail detector and visual autodock controller.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rail_autodock_node = greenhouse_pipe_rail_nav.rail_autodock_node:main",
            "video_image_publisher = greenhouse_pipe_rail_nav.video_image_publisher:main",
            "rail_offline_demo = greenhouse_pipe_rail_nav.offline_demo:main",
            "rail_benchmark_video = greenhouse_pipe_rail_nav.benchmark_video:main",
            "rail_calibrate_floor = greenhouse_pipe_rail_nav.floor_calibration:main",
            "rail_render_video = greenhouse_pipe_rail_nav.render_video:main",
        ],
    },
)
