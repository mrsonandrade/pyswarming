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

    leaderless_heading_consensus
    inverse_power
    spring
    force_law
    repulsive_force
    body_force
    inter_robot_spacing
    dissipative
    leader_following
    collision_avoidance
    attraction_alignment
    preferred_direction
    lennard_jones
    virtual_viscosity
    modified_attraction_alignment
    heading_consensus
    perimeter_defense
    environment_exploration
    aggregation
    alignment
    geofencing
    repulsion
    target
    area_coverage
    collective_navigation
    flocking

Combined Behaviors
------------------

   area_coverage = geofencing + repulsion
   collective_navigation = target + repulsion
   flocking = aggregation + repulsion + alignment
"""

__all__ = ['leaderless_heading_consensus', 'inverse_power', 'spring',
           'force_law', 'repulsive_force', 'body_force', 'inter_robot_spacing',
           'dissipative', 'leader_following', 'collision_avoidance',
           'attraction_alignment', 'preferred_direction', 'lennard_jones', 'virtual_viscosity',
           'modified_attraction_alignment', 'heading_consensus', 'perimeter_defense',
           'environment_exploration', 'aggregation', 'alignment', 'geofencing', 'repulsion',
           'target', 'area_coverage', 'collective_navigation', 'flocking']

import numpy as np

from numdifftools import Gradient


def leaderless_heading_consensus(theta_i, theta_j):
    """
    Calculate the new robot heading based on
    the "leaderless heading consensus algorithm"

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


def inverse_power(r_i, r_j, c_w, sigma_w):
    """
    Calculates an output force based on
    the "inverse-power force laws algorithm".
    This is a simplified version, i.e., the 
    coefficients between the particles are
    assumed to be the same.

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    c_w : numpy.array
        coefficients that depends on w. Where w is the
        number of inverse-power laws.

    sigma_w : numpy.array
        the inverse power coefficients (sigma_w>0)
        that depends on w.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """
    
    L = len(c_w)

    f_0 = np.zeros(3)
    f_i = np.zeros(3)

    for j in r_j:
        for w in range(L):
            f_0 += c_w[w] / np.power(np.linalg.norm(j - r_i), sigma_w[w])
        f_i += f_0 * ((j - r_i) / np.linalg.norm(j - r_i))

    return f_i


def spring(r_i, r_j, k, l):
    """
    Calculates an output force based on
    the "spring laws algorithm".
    This is a simplified version, i.e., the 
    coefficients between the particles are
    assumed to be the same.

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    k : float
        spring constant (k > 0).

    l : float
        desired distance between the robots.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """

    f_i = np.zeros(3)

    for j in r_j:
        f_i += k * (np.linalg.norm(j - r_i) - l) * ((j - r_i) / np.linalg.norm(j - r_i))

    return f_i


def force_law(r_i, r_j, G, m_i, m_j, p):
    """
    Calculates an output force based on
    the "force law algorithm".

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    G : float
        coefficient that acts like a gravitational
        constant.

    m_i : float
        mass of robot i.

    m_j : numpy.array
        array of masses of robots j.

    p : float
        user-defined power.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """

    N = len(r_j)

    f_i = np.zeros(3)

    for j in range(N):
        f_i += (G * m_i * m_j[j]) / np.power(np.linalg.norm(r_j[j] - r_i), p)

    return f_i


def repulsive_force(r_i, r_j, A_i, B_i, R_i, R_j):
    """
    Calculates an output force based on
    the "repulsive force algorithm".

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    A_i : float
        user-defined coefficient.

    B_i : float
        user-defined coefficient.

    R_i : float
        radii of the robot i.

    R_j : numpy.array
        array with the radii of the robots j.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """

    N = len(r_j)

    f_i = np.zeros(3)

    for j in range(N):
        r_ij = r_j[j] - r_i
        f_i += A_i * np.exp((R_i+R_j[j]+np.linalg.norm(r_ij))/B_i) * (r_ij / np.linalg.norm(r_ij))

    return f_i


def body_force(r_i, r_j, Lambda, R_i, R_j):
    """
    Calculates an output force based on
    the "body force algorithm".

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    Lambda : float
        user-defined coefficient.

    R_i : float
        radii of the robot i.

    R_j : numpy.array
        array with the radii of the robots j.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """

    def h(R_i, R_j, norm_r_ij):
        if norm_r_ij > (R_i + R_j):
            return 0
        else:
            return R_i + R_j + norm_r_ij

    N = len(r_j)

    f_i = np.zeros(3)

    for j in range(N):
        r_ij = r_j[j] - r_i
        f_i += Lambda * h(R_i, R_j[j], np.linalg.norm(r_ij)) * (r_ij / np.linalg.norm(r_ij))

    return f_i


def inter_robot_spacing(r_i, r_j, alpha, d_0):
    """
    Calculates an output force based on
    the "inter-robot spacing algorithm".

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
        user-defined coefficient.

    d_0 : float
        user-defined coefficient.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """
    
    f_i = np.zeros(3)

    for j in r_j:
        r_ij = j - r_i
        f_i += alpha * ((1.0/np.linalg.norm(r_ij)) - (d_0/np.power(np.linalg.norm(r_ij),2))) * (r_ij / np.linalg.norm(r_ij))

    return f_i


def dissipative(v_i, v_d, a):
    """
    Calculate a force based on
    the "dissipative force algorithm"

    Parameters
    ----------
    v_i : numpy.array
        array must have the robot velocity in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    v_d : numpy.array
        array must have the desired robot velocity in
        cartesian coordinates.

    a : float
        damping factor (a > 0).

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """

    f_i = - a * (v_i - v_d)

    return f_i


def leader_following(theta_i, theta_j, theta_0, b_i):
    """
    Calculate the new robot heading based on
    the "leader following algorithm"

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
    based on the "collision avoidance algorithm"

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
    new_theta_i : numpy.array
        array containing the new heading
    """
    
    new_theta_i = np.zeros(3)

    for j in r_j:
        new_theta_i -= ((j - r_i) / np.linalg.norm(j - r_i))

    return new_theta_i


def attraction_alignment(r_i, r_j, theta_j):
    """
    Calculate the desired heading of the robot
    based on the "attraction and alignment algorithm"

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
    new_theta_i : numpy.array
        array containing the new heading
    """
    
    new_theta_i = np.zeros(3)

    for j in r_j:
        new_theta_i += ((j - r_i) / np.linalg.norm(j - r_i))

    for j in theta_j:
        new_theta_i += (j / np.linalg.norm(j))

    new_theta_i = new_theta_i / np.linalg.norm(new_theta_i)

    return new_theta_i


def preferred_direction(theta_i, k_i, w=0.0):
    """
    Calculate the preferred direction of the robot
    based on the "preferred direction algorithm"

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
    new_theta_i : numpy.array
        array containing the new heading
    """
    
    new_theta_i = (theta_i + w*k_i) / np.linalg.norm(theta_i + w*k_i)

    return new_theta_i


def lennard_jones(r_i, r_j, epsilon, sigma):
    """
    Calculates an output force that produces
    lattice formations, based on the "Lennard-Jones
    potential algorithm"

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    r_j : numpy.array
        array must have the neighborhood positions in
        cartesian coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    epsilon : float
        depth of the potential well.

    sigma : float
        desired distance between the robots.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """
    
    N = len(r_j)

    f_i = np.zeros(3)

    for j in r_j:
        r_ij = (j - r_i)
        f_i += ((12.0*epsilon)/r_ij) * (np.power(sigma/r_ij, 12) - np.power(sigma/r_ij, 6)) * (r_ij / np.linalg.norm(r_ij))

    f_i = (1.0/N) * f_i

    return f_i


def virtual_viscosity(v_i, xi, xi_dot, xi_conv, xi_stab):
    """
    Calculates a virtual viscosity force
    based on the "virtual viscosity algorithm"

    Parameters
    ----------
    v_i : numpy.array
        array must have the robot velocity in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    xi : float
        damping factor.

    xi_dot : boolean
        boolean parameter to avoid residual oscillations
        after the convergence, when xi_dot = True.

    xi_conv : float
        damping factor that ensures the convergence.

    xi_stab : float
        damping factor of reference that depends on the
        orbit at which the reference frame is located.

    Returns
    -------
    f_i : numpy.array
        array containing the force
    """
    
    if xi_dot == True: # after convergence
        if xi < xi_stab:
            xi = xi_conv*np.exp(-xi/2.0)
        else:
            xi = 0

    f_i = - xi * v_i

    return f_i


def modified_attraction_alignment(r_i, r_j, theta_j, h_j):
    """
    Calculate the desired heading of the robot
    based on the "attraction and alignment algorithm"
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
    new_theta_i : numpy.array
        array containing the new heading
    """
    
    new_theta_i = np.zeros(3)

    for j,h in zip(r_j,h_j):
        new_theta_i += h*((j - r_i) / np.linalg.norm(j - r_i))

    for j,h in zip(theta_j,h_j):
        new_theta_i += h*(j / np.linalg.norm(j))

    new_theta_i = new_theta_i / np.linalg.norm(new_theta_i)

    return new_theta_i


def heading_consensus(theta_i, theta_j):
    """
    Calculate the new robot heading based on
    the "heading consensus algorithm"

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

    new_theta_i = np.zeros(3)

    for j in theta_j_u_i:
        new_theta_i += (1.0/(N + 1.0)) * (j)

    return new_theta_i


def perimeter_defense(r_i, r_j):
    """
    Calculate the new "heading" based on
    the "perimeter defense algorithm"

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
    g_i : numpy.array
        array containing g_i
    """
    
    g_i = np.zeros(3)

    for j in r_j:
        g_i += ((j - r_i) / np.power(np.linalg.norm(j - r_i), 2))

    return g_i


def environment_exploration(r_i, r_j, theta_j, H_i, T, r_0):
    """
    Calculate the new velocity of the robot
    based on the "environment exploration algorithm"

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

    H_i: integer
        binary heading consensus weight, being toward a goal (H_i = 1)
        or with reference to the others robots (H_i = 0).

    T : numpy.array
        array must have the target (goal) position in
        cartesian coordinates (i.e. np.asarray([x, y, z])).

    r_0 : float
        reference distance between the robots to create
        a set of robots within this range.

    Returns
    -------
    v_i : numpy.array
        array containing the new velocity v_i
    """

    N = len(r_j)

    beta_i = (T - r_i) / np.linalg.norm(T - r_i)

    term_2 = 0

    for j in r_j:

        gamma_ij = (j - r_i) / np.linalg.norm(j - r_i)

        term_2 += gamma_ij*((1-H_i)-(np.power(r_0,2)/np.power(np.linalg.norm(j - r_i),2)))

    R_i_u_i = []

    for j in range(len(r_j)):
        if np.linalg.norm(r_j[j] - r_i)<=r_0:
            R_i_u_i.append(j)

    term_3 = 0

    for j in R_i_u_i:

        term_3 += np.asarray([np.cos(theta_j[j][2]), np.sin(theta_j[j][2]), 0])

    v_i = H_i*beta_i + (1.0/N)*term_2 + (H_i/N)*term_3

    return v_i


def aggregation(r_i, r_j):
    """
    Calculates a nondimensional contribution
    based on the "aggregation algorithm"

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
    g_i : numpy.array
        array containing g_i
    """

    N = len(r_j)
    
    g_i_sum = np.zeros(3)

    for j in r_j:
        g_i_sum += ((j - r_i) / np.linalg.norm(j - r_i))

    g_i = (1.0/N) * g_i_sum

    return g_i


def alignment(v_i, v_j):
    """
    Calculates a nondimensional contribution
    based on the "alignment algorithm"

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
    g_i : numpy.array
        array containing g_i
    """

    N = len(v_j)
    
    g_i_sum = np.zeros(3)

    for j in v_j:
        g_i_sum += ((j - v_i) / np.linalg.norm(j - v_i))

    g_i = (1.0/N) * g_i_sum

    return g_i


def geofencing(r_i, A):
    """
    Calculates a nondimensional contribution
    based on the "geofencing algorithm"

    Parameters
    ----------
    r_i : numpy.array
        array must have the robot position in cartesian
        coordinates (i.e. np.asarray([x, y, z])).

    A : function
        function of the interested region.
        e.g. A = lambda x: np.sqrt(x[0]+x[1]+x[2])

    Returns
    -------
    g_i : numpy.array
        array containing contribution
    """

    gradF = Gradient(A)

    gradA = gradF([r_i[0],r_i[1],r_i[2]])

    g_i = - (1.0 / (1.0 + np.exp(-A([r_i[0],r_i[1],r_i[2]])))) * (gradA / np.linalg.norm(gradA))

    return g_i


def repulsion(r_i, r_j, alpha, d=2):
    """
    Calculates a nondimensional contribution
    based on the "repulsion algorithm"

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
    g_i : numpy.array
        array containing g_i
    """
    
    g_i = np.zeros(3)

    for j in r_j:
        g_i -= (np.power(alpha,d) / np.power(np.linalg.norm(j - r_i),d)) * ((j - r_i) / np.linalg.norm(j - r_i))

    return g_i


def target(r_i, T):
    """
    Calculates a nondimensional contribution
    based on the "target algorithm"

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
