from setuptools import find_packages, setup

setup(
    name="robotiq2F_tcp",
    version="0.0.1",
    url="https://github.com/airo-ugent/robotiq2F_tcp/.git",
    author="Thomas Lips",
    author_email="thomas.lips@ugent.be",
    description="Wrapper for TCP API of Robotiq URCap for controlling a robotiq2F gripper",
    packages=find_packages(),
)
