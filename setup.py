# TODO Test and deploy.
from setuptools import setup, find_packages
import sys, os
from setuptools.command.install import install as DistutilsInstall
from setuptools.command.egg_info import egg_info as EggInfo

setup(
    name = 'servorobots',
    version = '1.0',
    description = 'Robots with servomotors',
    maintainer = 'Charles Blouin',
    maintainer_email = '---@gmail.com',
    url = 'https://github.com/xxxxx',
    packages=[x for x in find_packages()],
    cmdclass = {
        'install': DistutilsInstall,
        'egg_info': EggInfo
        }
)