"""
``pyswarming.behaviors``
================

The PySwarming behaviors functions are based on researches of different authors,
for details, see the documentation available in two forms: docstrings provided
with the code, and a loose standing reference guide, available from
`the PySwarming homepage <https://github.com/mrsonandrade/pyswarming>`_..

Functions present in pyswarming.behaviors are listed below.

Behaviors
--------------------------

   aggregation
   alignment
   geofencing
   repulsion
   target
   area_coverage
   collective_navigation
   flocking
"""

__all__ = ['aggregation', 'alignment', 'geofencing', 'repulsion',
           'target', 'area_coverage', 'collective_navigation',
           'flocking']

import numpy as np

from auto_differentiation import gradient

def aggregation(r_i, r_j):
    """
    Calculate the aggregation nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    Returns
    -------
    b_A : numpy.array
        array containing contribution
    """

    N = len(r_j)
    
    b_A_sum = np.zeros(3)

    for j in r_j:
        b_A_sum += ((j - r_i) / np.linalg.norm(j - r_i))

    b_A = (1.0/N) * b_A_sum

    return b_A


def alignment(v_i, v_j):
    """
    Calculate the alignment nondimensional
    orientation contribution

    Parameters
    ----------
    v_i : numpy.array
        array must have the robot velocity in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    v_j : numpy.array
        array must have the neighborhood velocities in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    Returns
    -------
    b_AL : numpy.array
        array containing contribution
    """

    N = len(v_j)
    
    b_AL_sum = np.zeros(3)

    for j in v_j:
        b_AL_sum += ((j - v_i) / np.linalg.norm(j - v_i))

    b_AL = (1.0/N) * b_AL_sum

    return b_AL


def geofencing(r_i, A):
    """
    Calculate the geofencing nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    A : function
        function of the interested region.

    Returns
    -------
    b_G : numpy.array
        array containing contribution
    """

    gradA = np.asarray(gradient(A, (r_i[0],r_i[1],r_i[2])))

    b_G = - (1.0 / (1.0 + np.exp(-A(r_i[0],r_i[1],r_i[2])))) * (gradA / np.linalg.norm(gradA))

    return b_G


def repulsion(r_i, r_j, alpha, d=2):
    """
    Calculate the repulsion nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).
    
    alpha : float
        float parameter to determine the strength of
        the repulsion.
    
    d : integer
        integer > 1 parameter is the multipole order.

    Returns
    -------
    b_R : numpy.array
        array containing contribution
    """
    
    b_R = np.zeros(3)

    for j in r_j:
        b_R -= (np.power(alpha,d) / np.power(np.linalg.norm(j - r_i),d)) * ((j - r_i) / np.linalg.norm(j - r_i))

    return b_R


def target(r_i, T):
    """
    Calculate the target nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    T : numpy.array
        array must have the target position in
        cartesian coordinates (i.e. np.asarray([x, y, z])).

    Returns
    -------
    b_T : numpy.array
        array containing contribution
    """

    b_T = (T - r_i) / np.linalg.norm(T - r_i)

    return b_T

###################################################################
# Combined behaviors
###################################################################

def area_coverage(r_i, r_j, A, alpha, d=2):
    """
    Calculate the area coverage nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    A : function
        function of the interested region.
    
    alpha : float
        float parameter to determine the strength of
        the repulsion.
    
    d : integer
        integer > 1 parameter is the multipole order.

    Returns
    -------
    b_AC : numpy.array
        array containing contribution
    """

    b_AC = geofencing(r_i, A) + repulsion(r_i, r_j, alpha, d)

    return b_AC


def collective_navigation(r_i, r_j, T, alpha, d=2):
    """
    Calculate the collective navigation nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    T : numpy.array
        array must have the target position in
        cartesian coordinates (i.e. np.asarray([x, y, z])).
    
    alpha : float
        float parameter to determine the strength of
        the repulsion.
    
    d : integer
        integer > 1 parameter is the multipole order.

    Returns
    -------
    b_CN : numpy.array
        array containing contribution
    """

    b_CN = target(r_i, T) + repulsion(r_i, r_j, alpha, d)

    return b_CN


def flocking(r_i, r_j, v_i, v_j, alpha, d=2):
    """
    Calculate the flocking nondimensional
    orientation contribution

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    alpha : float
        float parameter to determine the strength of
        the repulsion.
    
    d : integer
        integer > 1 parameter is the multipole order.

    v_i : numpy.array
        array must have the robot velocity in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    v_j : numpy.array
        array must have the neighborhood velocities in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    Returns
    -------
    b_F : numpy.array
        array containing contribution
    """

    b_F = aggregation(r_i, r_j) + repulsion(r_i, r_j, alpha, d) + alignment(v_i, v_j)

    return b_F
