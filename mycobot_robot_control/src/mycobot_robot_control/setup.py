from glob import glob

from setuptools import find_packages, setup


package_name = "mycobot_robot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/docs", glob("docs/*.md")),
        (f"share/{package_name}/examples", glob("examples/*.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
        (f"share/{package_name}/provision", glob("provision/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workshop Team",
    maintainer_email="teacher@example.com",
    description="Student-facing myCobot 280 control scripts, examples, and tutorials.",
    license="Apache-2.0",
    scripts=[
        "mycobot_robot_control/go_home.py",
        "mycobot_robot_control/move_to_pose.py",
        "mycobot_robot_control/move_z.py",
        "mycobot_robot_control/pick_flow.py",
        "mycobot_robot_control/pick_service.py",
        "mycobot_robot_control/rotate_arm.py",
        "mycobot_robot_control/tactile_visualizer.py",
    ],
)
