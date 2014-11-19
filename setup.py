## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['marathon_gui'],
    scripts=['scripts/web_interface.py', 'scripts/card_interface.py', 'scripts/execute_task.py', 'scripts/insert_behaviour.py'],
    package_dir={'': 'src'})

setup(**setup_args)
