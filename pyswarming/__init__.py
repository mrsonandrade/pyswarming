"""
PySwarming

=====
A toolkit for swarm robotics

How to use the documentation
----------------------------

Documentation is available in two forms: docstrings provided
with the code, and a loose standing reference guide, available from
`the PySwarming homepage <https://github.com/mrsonandrade/pyswarming>`_.

Available subpackages
---------------------

behaviors
    Collection of swarming behaviors based on different research articles.

swarm
    Allow the creation of simple virtual swarms.
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

# To get sub-modules
from . import behaviors
from . import swarm

modules = [behaviors.__all__.copy(), swarm.__all__.copy()]

__all__ = []
for module_i in modules:
    for module_j in module_i:
        __all__.append(module_j)

name = "pyswarming"
__version__ = "1.1.4"
__author__ = "Emerson Martins de Andrade, Antonio Carlos Fernandes and Joel Sena Sales Jr"
__author_email__ = "mrson@oceanica.ufrj.br"
