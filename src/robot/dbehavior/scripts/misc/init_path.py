"""Set up paths."""

from os import path
import sys


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)


this_dir = path.dirname(__file__)

# Add lib to PYTHONPATH
lib_path = path.join(this_dir, '..')
add_path(lib_path)
