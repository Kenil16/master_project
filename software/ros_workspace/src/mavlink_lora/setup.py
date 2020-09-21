from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/gcs_simple.py', 'scripts/gcs_keypress.py', 'scripts/param_list_get.py', 'scripts/mission_download.py', 'scripts/mission_set_current.py', 'scripts/req_data_stream.py'],)
setup(**d)

