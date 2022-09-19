.. pyswarming documentation master file.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to pyswarming's documentation!
++++++++++++++++++++++++++++++++++++

.. image:: ../readme_pics/logo.png
   :alt: pyswarming
   :align: left


Introduction
============

``pyswarming`` is a research toolkit for Swarm Robotics.

The package is currently maintained by `@mrsonandrade <http://github.com/mrsonandrade>`_.

https://github.com/mrsonandrade/pyswarming

Install
=======
You can install ``pyswarming`` from PyPI using pip (**Recommended**)::

   pip install pyswarming


Dependencies
============

``pyswarming``'s dependencies are: ``numpy``.


Examples
========
Considering a swarm of robots, they can show different behaviors by using ``pyswarming``. The following codes are simplified implementations, for detailed ones, see the `Examples <https://github.com/mrsonandrade/pyswarming/tree/main/Examples>`_ folder.::

   import pyswarming.behaviors as ps
   import numpy as np

Target
------
::

   # define the robot (x, y, z) position
   r_i = np.asarray([0., 0., 0.])

   # set the robot linear velocity
   s_i = 1.0

   # define a target (x, y, z) position
   T = np.asarray([8., 8., 0.])

   for t in range(15):

      # print the robot (x, y, z) position
      print(r_i)

      # update the robot (x, y, z) position
      r_i += s_i*ps.target(r_i, T)


.. image:: ../readme_pics/Target.gif
   :alt: target

Aggregation
-----------
::

   # define each robot (x, y, z) position
   r = np.asarray([[8., 8., 0.],
                  [-8., 8., 0.],
                  [8., -8., 0.],
                  [-8., -8., 0.]])

   # set the robot linear velocity
   s_i = 1.0

   for t in range(15):

      # print the robot (x, y, z) positions
      print(r)

      # update the robot (x, y, z) positions
      for r_ind in range(len(r)):
         r_i = r[r_ind]
         r_j = np.delete(r, np.array([r_ind]), axis=0)
         r[r_ind] += s_i*ps.aggregation(r_i, r_j)

.. image:: ../readme_pics/Aggregation.gif

Repulsion
---------
::

   # define each robot (x, y, z) position
   r = np.asarray([[1., 1., 0.],
                  [-1., 1., 0.],
                  [1., -1., 0.],
                  [-1., -1., 0.]])

   # set the robot linear velocity
   s_i = 1.0

   for t in range(15):

      # print the robot (x, y, z) positions
      print(r)

      # update the robot (x, y, z) positions
      for r_ind in range(len(r)):
         r_i = r[r_ind]
         r_j = np.delete(r, np.array([r_ind]), axis=0)
         r[r_ind] += s_i*ps.repulsion(r_i, r_j, 3.0)

.. image:: ../readme_pics/Repulsion.gif

Aggregation + Repulsion
-----------------------
::

   # define each robot (x, y, z) position
   r = np.asarray([[8., 8., 0.],
                  [-8., 8., 0.],
                  [8., -8., 0.],
                  [-8., -8., 0.]])

   # set the robot linear velocity
   s_i = 1.0

   for t in range(15):

      # print the robot (x, y, z) positions
      print(r)

      # update the robot (x, y, z) positions
      for r_ind in range(len(r)):
         r_i = r[r_ind]
         r_j = np.delete(r, np.array([r_ind]), axis=0)
         r[r_ind] += s_i*(ps.aggregation(r_i, r_j) + ps.repulsion(r_i, r_j, 5.0))

.. image:: ../readme_pics/AggregationRepulsion.gif







.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Contents:

   Introduction <self>
   Examples
   API Reference <pyswarming>
   All Functions <functions>
   Contribution
   Acknowledgements




