import pyswarming.behaviors as ps
import numpy as np

'''
    #   Tested      Algorithm

    1   yes         leaderless_heading_consensus
    2   yes         inverse_power
    3   yes         spring
    4   yes         force_law
    5   yes         repulsive_force
    6   yes         body_force
    7   yes         inter_robot_spacing
    8   yes         dissipative
    9   yes         leader_following
    10  yes         collision_avoidance
    11  yes         attraction_alignment
    12  yes         preferred_direction
    13  yes         lennard_jones
    14  yes         virtual_viscosity (14_1 and 14_2)
    15  yes         modified_attraction_alignment
    16  yes         heading_consensus
    17  yes         perimeter_defense
    18  yes         environment_exploration (18_1 and 18_2)
    19  yes         aggregation
    20  yes         alignment
    21  yes         geofencing
    22  yes         repulsion
    23  yes         target
'''

# 1
def test_leaderless_heading_consensus():
    # leaderless coordination with four robots
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta) # initial state
    for theta_ind in range(len(theta)):
        theta_i = theta[theta_ind]
        theta_j = np.delete(theta, np.array([theta_ind]), axis=0)
        theta[theta_ind] = ps.leaderless_heading_consensus(theta_i, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).any() == True

# 2
def test_inverse_power():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    c_w = np.asarray([1.0, -1.0])
    sigma_w = np.asarray([1.0, 2.0])
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.inverse_power(r_i, r_j, c_w, sigma_w)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 3
def test_spring():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    k = 10.0
    l = 5.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.spring(r_i, r_j, k, l)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 4
def test_force_law():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    G = 10.0
    m_i = 1.0
    m_j = np.ones(4, dtype=float)
    p = 2.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.force_law(r_i, r_j, G, m_i, m_j, p)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 5
def test_repulsive_force():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    A_i = 1.0
    B_i = 1.0
    R_i = 5.0
    R_j = 5.0*np.ones(4, dtype=float)
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.repulsive_force(r_i, r_j, A_i, B_i, R_i, R_j)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 6
def test_body_force():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    Lambda = 1.0
    R_i = 30.0
    R_j = 30.0*np.ones(4, dtype=float)
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.body_force(r_i, r_j, Lambda, R_i, R_j)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 7
def test_inter_robot_spacing():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    alpha = 1.0
    d_0 = 5.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.inter_robot_spacing(r_i, r_j, alpha, d_0)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 8
def test_dissipative():
    # test with four robots
    v = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    v_d = 4.0
    a = 1.0
    for v_ind in range(len(v)):
        v_i = v[v_ind]
        f[v_ind] += ps.dissipative(v_i, v_d, a)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 9
def test_leader_following_1():
    # leader following with four robots
    theta_0 = np.asarray([0.78, 0.78, 0.78])
    b_i = 1
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta) # initial state
    for theta_ind in range(len(theta)):
        theta_i = theta[theta_ind]
        theta_j = np.delete(theta, np.array([theta_ind]), axis=0)
        theta[theta_ind] = ps.leader_following(theta_i, theta_j, theta_0, b_i)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True

# 10
def test_collision_avoidance_1():
    # collision avoidance with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.collision_avoidance(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True

# 11
def test_attraction_alignment_1():
    # attraction and alignment with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta) # initial state
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        theta_j = np.delete(theta, np.array([r_ind]), axis=0)
        theta[r_ind] = ps.attraction_alignment(r_i, r_j, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True

# 12
def test_preferred_direction_1():
    # preferred direction with one robot
    theta_i = np.asarray([0., 0., 0.])
    theta_i0 = np.copy(theta_i)
    w = 1.0
    k_i = np.asarray([0.78, 0.78, 0.78])
    theta_i = ps.preferred_direction(theta_i, k_i, w)
    assert type(theta_i) == np.ndarray
    assert len(theta_i) > 0
    assert (theta_i!=theta_i0).all() == True

# 13
def test_lennard_jones():
    # test with four robots
    r = np.asarray([[8., 5., 8.],
                    [-5., 6., 7.],
                    [7., -8., 6.],
                    [-6., -7., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    epsilon = 1.0
    sigma = 2.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += ps.lennard_jones(r_i, r_j, epsilon, sigma)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 14_1
def test_virtual_viscosity_1():
    # test with four robots
    v = np.asarray([[8., 5., 8.],
                    [-5., 6., 7.],
                    [7., -8., 6.],
                    [-6., -7., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    xi = 0.2
    xi_dot = False
    xi_conv = 0.5
    xi_stab = 0.7
    for v_ind in range(len(v)):
        v_i = v[v_ind]
        f[v_ind] += ps.virtual_viscosity(v_i, xi, xi_dot, xi_conv, xi_stab)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 14_2
def test_virtual_viscosity_2():
    # test with four robots
    v = np.asarray([[8., 5., 8.],
                    [-5., 6., 7.],
                    [7., -8., 6.],
                    [-6., -7., 5.]])
    f = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    f0 = np.copy(f) # initial state
    xi = 0.2
    xi_dot = True
    xi_conv = 0.5
    xi_stab = 0.7
    for v_ind in range(len(v)):
        v_i = v[v_ind]
        f[v_ind] += ps.virtual_viscosity(v_i, xi, xi_dot, xi_conv, xi_stab)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True

# 15
def test_modified_attraction_alignment_1():
    # modified attraction and alignment with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta) # initial state
    h_j = np.asarray([1.0, 1.0, 1.0, 1.0])
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        theta_j = np.delete(theta, np.array([r_ind]), axis=0)
        theta[r_ind] = ps.modified_attraction_alignment(r_i, r_j, theta_j, h_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True

# 16
def test_heading_consensus_1():
    # heading consensus with four robots
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta) # initial state
    for theta_ind in range(len(theta)):
        theta_i = theta[theta_ind]
        theta_j = np.delete(theta, np.array([theta_ind]), axis=0)
        theta[theta_ind] = ps.heading_consensus(theta_i, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).any() == True

# 17
def test_perimeter_defense_1():
    # perimeter defense with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.perimeter_defense(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True

# 18_1
def test_environment_exploration_1():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    v = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    v0 = np.copy(v) # initial state
    H_i = 1
    T = np.asarray([0., 0., 0.])
    r_0 = 2.0
    for v_ind in range(len(v)):
        r_i = r[v_ind]
        r_j = np.delete(r, np.array([v_ind]), axis=0)
        theta_j = np.delete(theta, np.array([v_ind]), axis=0)
        v[v_ind] += ps.environment_exploration(r_i, r_j, theta_j, H_i, T, r_0)
    assert type(v) == np.ndarray
    assert len(v) > 0
    assert (v!=v0).all() == True

# 18_2
def test_environment_exploration_2():
    # test with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    v = np.asarray([[0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.],
                    [0., 0., 0.]])
    v0 = np.copy(v) # initial state
    H_i = 0
    T = np.asarray([0., 0., 0.])
    r_0 = 2.0
    for v_ind in range(len(v)):
        r_i = r[v_ind]
        r_j = np.delete(r, np.array([v_ind]), axis=0)
        theta_j = np.delete(theta, np.array([v_ind]), axis=0)
        v[v_ind] += ps.environment_exploration(r_i, r_j, theta_j, H_i, T, r_0)
    assert type(v) == np.ndarray
    assert len(v) > 0
    assert (v!=v0).all() == True

# 19
def test_aggregation_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True

# 20
def test_alignment_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True

# 21
def test_geofencing_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True

# 22
def test_repulsion_1():
    # repulsion with four robots
    r = np.asarray([[1., 1., 0.1],
                [-1., 1., 0.2],
                [1., -1., 0.3],
                [-1., -1., 0.4]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.repulsion(r_i, r_j, 3.0)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True

# 23
def test_target_1():
    # target with one robot
    r_i = np.asarray([0., 0., 0.])
    r_i0 = np.copy(r_i)
    s_i = 1.0 # linear speed
    T = np.asarray([8., 8., 8.])
    r_i = s_i*ps.target(r_i, T)
    assert type(r_i) == np.ndarray
    assert len(r_i) > 0
    assert (r_i!=r_i0).all() == True

