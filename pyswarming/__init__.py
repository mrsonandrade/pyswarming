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
    Collection of swarming behaviors.
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

# To get sub-modules
from . import behaviors

__all__ = behaviors.__all__.copy()

name = "pyswarming"
__version__ = "1.0.0"
__author__ = "Emerson de Andrade, Antonio Fernandes and Joel Sales Jr"
__author_email__ = "mrson@oceanica.ufrj.br"
