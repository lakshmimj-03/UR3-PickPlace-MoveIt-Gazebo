from setuptools import find_packages
from setuptools import setup

setup(
    name='ur3_pick_place',
    version='0.0.1',
    packages=find_packages(
        include=('ur3_pick_place', 'ur3_pick_place.*')),
)
