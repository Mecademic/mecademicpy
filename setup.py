#!/usr/bin/env python3
""" Setup for this module """
import sys

import setuptools

with open('README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()

dependencies = ['dataclasses_json>=0.5.4', 'deprecation', 'pandas', 'requests', 'pyyaml']
if sys.version_info < (3, 8, 0):
    dependencies.append('importlib_metadata')

setuptools.setup(name='mecademicpy',
                 version='2.3.0',
                 author='Mecademic',
                 author_email='support@mecademic.com',
                 license='MIT',
                 description='A package to control the Mecademic robots through python',
                 long_description=long_description,
                 long_description_content_type='text/markdown',
                 url='https://github.com/Mecademic/mecademicpy',
                 packages=setuptools.find_packages(),
                 include_package_data=True,
                 classifiers=[
                     'Programming Language :: Python :: 3',
                     'License :: OSI Approved :: MIT License',
                     'Operating System :: OS Independent',
                 ],
                 python_requires='>=3.7',
                 install_requires=dependencies)
