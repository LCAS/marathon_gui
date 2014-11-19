## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['marathon_gui'],
    scripts=['scripts/robblog_node.py', 'scripts/set_alias.py', 'scripts/set_safe_points.py', 'scripts/web_interface.py'],
    package_dir={'': 'src'})

setup(**setup_args)
