import sys
from skbuild import setup
from setuptools import find_packages

setup(
    name="physics_mujoco",
    version="0.0.1",
    description="physics based on mujoco",
    author="Riften",
    license="BSD",
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    cmake_install_dir="python/physics_mujoco",
    include_package_data=True,
    extras_require={"test": ["pytest"]},
    python_requires=">=3.6",
)