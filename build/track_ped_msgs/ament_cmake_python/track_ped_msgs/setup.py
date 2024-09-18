from setuptools import find_packages
from setuptools import setup

setup(
    name='track_ped_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('track_ped_msgs', 'track_ped_msgs.*')),
)
