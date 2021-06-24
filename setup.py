from setuptools import setup, find_packages


# Version meaning (X.Y.Z)
# X: Major version (e.g. vastly different scene, platform, etc)
# Y: Minor version (e.g. new tasks, major changes to existing tasks, etc)
# Z: Patch version (e.g. small changes to tasks, bug fixes, etc)

setup(
    name='ros_panda_interface',
    version='1.0.0',
    author='Firas Al-Hafez',
    author_email="f.al-hafez@posteo.de",
    packages=find_packages(),
)
