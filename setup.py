#!/usr/bin/env python
'ROS Package build + installation script'

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_options = generate_distutils_setup()
setup_options['packages'] = ['baxter_projectyouth']
setup_options['package_dir'] = {'': 'src'}

setup(**setup_options)
