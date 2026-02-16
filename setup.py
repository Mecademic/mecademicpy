#!/usr/bin/env python3
""" Setup for this module """
import sys

import setuptools

with open('README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()

# These dependencies are absolutely necessary for running mecademicpy and controlling a robot
minimal_dependencies = []
if sys.version_info < (3, 8, 0):
    minimal_dependencies.append('importlib_metadata')

# These dependencies are only necessary for method UpdateRobot
update_robot_dependencies = ['requests']

# These dependencies are only necessary if using trajectory logger
trajectory_logger_dependencies = ['dataclasses_json>=0.5.4', 'pandas']

all_dependencies = minimal_dependencies + update_robot_dependencies + trajectory_logger_dependencies

setuptools.setup(name='mecademicpy',
                 version='3.0.0',
                 author='Mecademic',
                 author_email='support@mecademic.com',
                 license='MIT',
                 description='A package to control the Mecademic robots through python',
                 long_description=long_description,
                 long_description_content_type='text/markdown',
                 url='https://github.com/Mecademic/mecademicpy',
                 packages=setuptools.find_packages(include=["mecademicpy*"]),
                 include_package_data=True,
                 classifiers=[
                     'Programming Language :: Python :: 3',
                     'License :: OSI Approved :: MIT License',
                     'Operating System :: OS Independent',
                 ],
                 python_requires='>=3.8',
                 install_requires=minimal_dependencies,
                 extras_require={
                     'basic': [minimal_dependencies],
                     'update_robot': minimal_dependencies + update_robot_dependencies,
                     'trajectory_logger': minimal_dependencies + trajectory_logger_dependencies,
                     'full': all_dependencies
                 })
