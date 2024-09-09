from setuptools import find_packages
from setuptools import setup

setup(
    name='cnn_msgs',
    version='1.1.3',
    packages=find_packages(
        include=('cnn_msgs', 'cnn_msgs.*')),
)
