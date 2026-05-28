from setuptools import find_packages, setup


package_name = "yolo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo_pipeline.launch.py"]),
        ("share/" + package_name + "/config", ["config/yolo_detector.yaml"]),
        (
            "share/" + package_name + "/config",
            ["config/survivor_pose_to_marker.yaml"],
        ),
        ("share/" + package_name + "/models", ["yolov8s.pt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokey@todo.todo",
    description="YOLOv8 RGB-D survivor detection, localization, and RViz marker publishing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_detector = yolo.yolo_detector:main",
            "yolo_detector_auto = yolo.launch_yolo_detector:main",
            "survivor_pose_to_marker = yolo.survivor_pose_to_marker:main",
        ],
    },
)
