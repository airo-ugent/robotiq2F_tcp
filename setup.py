from setuptools import find_packages, setup

setup(
    name="robotiq2f",
    version="0.0.2",
    url="https://github.com/airo-ugent/robotiq2F/.git",
    author="Thomas Lips",
    author_email="thomas.lips@ugent.be",
    description="Driver for controlling a robotiq2F gripper using the Robotiq URCap TCP interface",
    packages=find_packages(),
)
