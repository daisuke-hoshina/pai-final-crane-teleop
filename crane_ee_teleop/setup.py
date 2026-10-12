from setuptools import find_packages, setup

package_name = "crane_ee_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daisuke-Hoshina",
    maintainer_email="21813243+daisuke-hoshina@users.noreply.github.com",
    description="Hand-space (end-effector) keyboard teleoperation for CRANE+ V2 in Gazebo",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
