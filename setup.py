from setuptools import find_packages, setup

setup(
    name="robotiq2f",
    version="0.0.2",
    url="https://github.com/airo-ugent/robotiq2F/.git",
    author="Thomas Lips",
    author_email="thomas.lips@ugent.be",
    description="Drivers for controlling a robotiq2F gripper over a number of physical interfaces",
    packages=find_packages(),
)
