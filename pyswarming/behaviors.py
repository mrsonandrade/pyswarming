"""
``pyswarming.behaviors``
========================

The PySwarming behaviors functions are based on researches of different authors,
for details, see the documentation available in two forms: docstrings provided
with the code, and a loose standing reference guide, available from
`the PySwarming homepage <https://github.com/mrsonandrade/pyswarming>`_..

Functions present in pyswarming.behaviors are listed below.

Behaviors
---------

   leaderless_coordination
   leader_following
   collision_avoidance
   attraction_alignment
   preferred_direction
   modified_attraction_alignment
   heading_consensus
   perimeter_defense
   aggregation
   alignment
   geofencing
   repulsion
   target

Combined Behaviors
------------------

   area_coverage = geofencing + repulsion
   collective_navigation = target + repulsion
   flocking = aggregation + repulsion + alignment
"""

__all__ = ['leaderless_coordination', 'leader_following',
            'collision_avoidance', 'attraction_alignment',
            'preferred_direction', 'modified_attraction_alignment',
            'heading_consensus', 'perimeter_defense',
            'aggregation', 'alignment', 'geofencing', 'repulsion',
            'target', 'area_coverage', 'collective_navigation',
            'flocking']

import numpy as np

from auto_differentiation import gradient


def leaderless_coordination(theta_i, theta_j):
    """
    Calculate the new robot heading based on
    the leaderless coordination algorithm

    Parameters
    ----------
    theta_i : numpy.array
        array must have the robot orientation in euler
        angles (i.e. np.asarray([roll, pitch, yaw])).

    theta_j : numpy.array
        array must have the neighborhood orientations
        in euler angles (i.e. np.asarray([[roll1, pitch1, yaw1],
        [roll2, pitch2, yaw2], ..., [rollN, pitchN, yawN]])).

    Returns
    -------
    new_theta_i : numpy.array
        array containing the new heading
    """

    N = len(theta_j)

    new_theta_i = (1.0/(1.0 + N)) * (theta_i + np.sum(theta_j, axis=0))

    return new_theta_i


def leader_following(theta_i, theta_j, theta_0, b_i):
    """
    Calculate the new robot heading based on
    the leader following algorithm

    Parameters
    ----------
    theta_i : numpy.array
        array must have the robot orientation in euler
        angles (i.e. np.asarray([roll, pitch, yaw])).

    theta_j : numpy.array
        array must have the neighborhood orientations
        in euler angles (i.e. np.asarray([[roll1, pitch1, yaw1],
        [roll2, pitch2, yaw2], ..., [rollN, pitchN, yawN]])).

    theta_0 : numpy.array
        array must have the leader robot orientation in euler
        angles (i.e. np.asarray([roll, pitch, yaw])).

    b_i: integer
        integer value where the value is 1 if the leader robot
        is a neighbor or the robot_i, and 0 otherwise.

    Returns
    -------
    new_theta_i : numpy.array
        array containing the new heading
    """

    N = len(theta_j)

    new_theta_i = (1.0/(1 + N + b_i)) * (theta_i + np.sum(theta_j, axis=0) + b_i*theta_0)

    return new_theta_i


def collision_avoidance(r_i, r_j):
    """
    Calculate the desired heading of the robot
    based on the collision avoidance algorithm

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
    theta_i : numpy.array
        array containing the desired heading
    """
    
    theta_i = np.zeros(3)

    for j in r_j:
        theta_i -= ((j - r_i) / np.linalg.norm(j - r_i))

    return theta_i


def attraction_alignment(r_i, r_j, theta_j):
    """
    Calculate the desired heading of the robot
    based on the attraction and alignment algorithm

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    theta_j : numpy.array
        array must have the neighborhood orientations
        in euler angles (i.e. np.asarray([[roll1, pitch1, yaw1],
        [roll2, pitch2, yaw2], ..., [rollN, pitchN, yawN]])).

    Returns
    -------
    theta_i : numpy.array
        array containing the desired heading
    """
    
    theta_i = np.zeros(3)

    for j in r_j:
        theta_i += ((j - r_i) / np.linalg.norm(j - r_i))

    for j in theta_j:
        theta_i += (j / np.linalg.norm(j))

    theta_i = theta_i / np.linalg.norm(theta_i)

    return theta_i


def preferred_direction(theta_i, k_i, w=0.0):
    """
    Calculate the preferred direction of the robot
    based on the preferred direction algorithm

    Parameters
    ----------
    theta_i : numpy.array
        array must have the robot orientation in euler
        angles (i.e. np.asarray([roll, pitch, yaw])).

    k_i : numpy.array
        array must have the a preferred direction "k" in
        euler angles (i.e. np.asarray([roll, pitch, yaw])).

    w : float
        float number giving a weight > 0 for the preferred
        direction.

    Returns
    -------
    p_theta_i : numpy.array
        array containing the preferred direction
    """
    
    p_theta_i = (theta_i + w*k_i) / np.linalg.norm(theta_i + w*k_i)

    return p_theta_i


def modified_attraction_alignment(r_i, r_j, theta_j, h_j):
    """
    Calculate the desired heading of the robot
    based on the attraction and alignment algorithm
    by considering the "social importance" factor "h"

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    theta_j : numpy.array
        array must have the neighborhood orientations
        in euler angles (i.e. np.asarray([[roll1, pitch1, yaw1],
        [roll2, pitch2, yaw2], ..., [rollN, pitchN, yawN]])).

    h_j : numpy.array
        array must have the neighborhood "social importance"
        factor (i.e. np.asarray([h_j1, h_j2, ..., h_jN])).

    Returns
    -------
    theta_i : numpy.array
        array containing the desired heading
    """
    
    theta_i = np.zeros(3)

    for j,h in zip(r_j,h_j):
        theta_i += h*((j - r_i) / np.linalg.norm(j - r_i))

    for j,h in zip(theta_j,h_j):
        theta_i += h*(j / np.linalg.norm(j))

    theta_i = theta_i / np.linalg.norm(theta_i)

    return theta_i


def heading_consensus(theta_i, theta_j):
    """
    Calculate the new robot heading based on
    the heading consensus algorithm

    Parameters
    ----------
    theta_i : numpy.array
        array must have the robot orientation in euler
        angles (i.e. np.asarray([roll, pitch, yaw])).

    theta_j : numpy.array
        array must have the neighborhood orientations
        in euler angles (i.e. np.asarray([[roll1, pitch1, yaw1],
        [roll2, pitch2, yaw2], ..., [rollN, pitchN, yawN]])).

    Returns
    -------
    new_theta_i : numpy.array
        array containing the new heading
    """

    theta_j_u_i = np.concatenate((theta_j, np.asarray([theta_i])), axis=0)

    N = len(theta_j)

    new_theta_i = (1.0/(N + 1.0)) * (np.sum(theta_j_u_i, axis=0))

    return new_theta_i


def perimeter_defense(r_i, r_j):
    """
    Calculate the new robot position based on
    the perimeter defense algorithm

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
    new_r_i : numpy.array
        array containing the new r_i
    """
    
    new_r_i = np.zeros(3)

    for j in r_j:
        new_r_i += ((j - r_i) / np.power(np.linalg.norm(j - r_i), 2))

    return new_r_i


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
