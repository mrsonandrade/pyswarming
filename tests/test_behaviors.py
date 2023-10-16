import pyswarming.behaviors as pb
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
    24  yes         area_coverage
    25  yes         collective_navigation
    26  yes         flocking
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
        theta[theta_ind] = pb.leaderless_heading_consensus(theta_i, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).any() == True
    theta_expected = np.array([[ 0.       ,  0.       ,  0.78],
                               [-0.195    , -0.195    ,  0.78],
                               [-0.04875  , -0.43875  ,  0.78],
                               [-0.2559375, -0.3534375,  0.78]])
    assert np.isclose(theta, theta_expected, atol=1e-3).all() == True

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
        f[r_ind] += pb.inverse_power(r_i, r_j, c_w, sigma_w)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-0.16951246, -0.22689672, -0.0389566],
                           [ 0.12950178, -0.22876192, -0.0205013],
                           [-0.22945604,  0.12862721,  0.0017374],
                           [ 0.18760583,  0.12865466,  0.0278072]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        f[r_ind] += pb.spring(r_i, r_j, k, l)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-235.04873473, -235.33746982,  -44.10772965],
                           [ 234.77650822, -235.06524331,  -14.70962365],
                           [-234.77650822,  235.06524331,   14.70962365],
                           [ 235.04873473,  235.33746982,   44.10772965]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        f[r_ind] += pb.force_law(r_i, r_j, G, m_i, m_j, p)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[0.0965659, 0.0965659, 0.0965659],
                           [0.0968652, 0.0968652, 0.0968652],
                           [0.0968652, 0.0968652, 0.0968652],
                           [0.0965659, 0.0965659, 0.0965659]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
    A_i = 10.0
    B_i = 100.0
    R_i = 5.0
    R_j = 5.0*np.ones(4, dtype=float)
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += pb.repulsive_force(r_i, r_j, A_i, B_i, R_i, R_j)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-22.68138629, -22.61848481,  -4.24489724],
                           [ 22.73974512, -22.67684364,  -1.41337138],
                           [-22.73974512,  22.67684364,   1.41337138],
                           [ 22.68138629,  22.61848481,   4.24489724]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
    Lambda = 0.1
    R_i = 20.0
    R_j = 20.0*np.ones(4, dtype=float)
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += pb.body_force(r_i, r_j, Lambda, R_i, R_j)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[ -9.99610122,  -9.97300241,  -1.87138163],
                           [ 10.01787934,  -9.99478054,  -0.62323011],
                           [-10.01787934,   9.99478054,   0.62323011],
                           [  9.99610122,   9.97300241,   1.87138163]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        f[r_ind] += pb.inter_robot_spacing(r_i, r_j, alpha, d_0)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-0.06682243, -0.06643918, -0.0124813],
                           [ 0.06714337, -0.06676012, -0.0041485],
                           [-0.06714337,  0.06676012,  0.0041485],
                           [ 0.06682243,  0.06643918,  0.0124813]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        f[v_ind] += pb.dissipative(v_i, v_d, a)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-4., -4., -4.],
                           [12., -4., -3.],
                           [-4., 12., -2.],
                           [12., 12., -1.]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        theta[theta_ind] = pb.leader_following(theta_i, theta_j, theta_0, b_i)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True
    theta_expected = np.array([[ 0.156  ,  0.156  ,  0.78],
                               [ 0.0312 ,  0.0312 ,  0.78],
                               [ 0.19344, -0.11856,  0.78],
                               [ 0.07612,  0.01372,  0.78]])
    assert np.isclose(theta, theta_expected, atol=1e-3).all() == True

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
        r[r_ind] += s_i*pb.collision_avoidance(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 9.69902531,  9.6932506 ,  8.31784541],
                           [-9.69914894,  9.60372134,  7.09426836],
                           [ 9.61160337, -9.69158037,  5.88927766],
                           [-9.59984268, -9.59383808,  4.70078007]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

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
        theta[r_ind] = pb.attraction_alignment(r_i, r_j, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True
    theta_expected = np.array([[-0.6480828 , -0.64643874,  0.40262345],
                               [ 0.26856791, -0.88977163,  0.36902265],
                               [-0.86936125, -0.13552146,  0.47523147],
                               [ 0.27644854,  0.01321525,  0.96093786]])
    assert np.isclose(theta, theta_expected, atol=1e-3).all() == True

# 12
def test_preferred_direction_1():
    # preferred direction with one robot
    theta_i = np.asarray([0., 0., 0.])
    theta_i0 = np.copy(theta_i)
    w = 1.0
    k_i = np.asarray([0.78, 0.78, 0.78])
    theta_i = pb.preferred_direction(theta_i, k_i, w)
    assert type(theta_i) == np.ndarray
    assert len(theta_i) > 0
    assert (theta_i!=theta_i0).all() == True
    theta_i_expected = np.array([0.57735027, 0.57735027, 0.57735027])
    assert np.isclose(theta_i, theta_i_expected, atol=1e-3).all() == True

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
    sigma = 1.2
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        f[r_ind] += pb.lennard_jones(r_i, r_j, epsilon, sigma)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[1.7982436 , 1.81394895, 1.79958809],
                           [1.79824347, 1.81394908, 3.08499675],
                           [1.79824347, 1.81394908, 3.08499675],
                           [1.7982436 , 1.81394895, 1.79958809]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        f[v_ind] += pb.virtual_viscosity(v_i, xi, xi_dot, xi_conv, xi_stab)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-1.6, -1.0, -1.6],
                           [ 1.0, -1.2, -1.4],
                           [-1.4,  1.6, -1.2],
                           [ 1.2,  1.4, -1.0]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        f[v_ind] += pb.virtual_viscosity(v_i, xi, xi_dot, xi_conv, xi_stab)
    assert type(f) == np.ndarray
    assert len(f) > 0
    assert (f!=f0).all() == True
    f_expected = np.array([[-3.61934967, -2.26209355, -3.61934967],
                           [ 2.26209355, -2.71451225, -3.16693096],
                           [-3.16693096,  3.61934967, -2.71451225],
                           [ 2.71451225,  3.16693096, -2.26209355]])
    assert np.isclose(f, f_expected, atol=1e-3).all() == True

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
        theta[r_ind] = pb.modified_attraction_alignment(r_i, r_j, theta_j, h_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True
    theta_expected = np.array([[-0.6480828 , -0.64643874,  0.40262345],
                               [ 0.26856791, -0.88977163,  0.36902265],
                               [-0.86936125, -0.13552146,  0.47523147],
                               [ 0.27644854,  0.01321525,  0.96093786]])
    assert np.isclose(theta, theta_expected, atol=1e-3).all() == True

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
        theta[theta_ind] = pb.heading_consensus(theta_i, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).any() == True
    theta_expected = np.array([[ 0.      ,  0.      ,  0.78],
                               [-0.195   , -0.195   ,  0.78],
                               [-0.04875 , -0.43875 ,  0.78],
                               [-0.255937, -0.353437,  0.78]])
    assert np.isclose(theta, theta_expected, atol=1e-3).all() == True

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
        r[r_ind] += s_i*pb.perimeter_defense(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 7.90703302,  7.90775137,  7.98265848],
                           [-7.90618673,  7.90690928,  6.99422699],
                           [ 7.90600985, -7.90672818,  6.00578459],
                           [-7.90612994, -7.90685232,  5.01751034]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

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
        v[v_ind] += pb.environment_exploration(r_i, r_j, theta_j, H_i, T, r_0)
    assert type(v) == np.ndarray
    assert len(v) > 0
    assert (v!=v0).all() == True
    v_expected = np.array([[-0.57037839, -0.57046775, -0.57605421],
                           [ 0.59430282, -0.59439218, -0.52572501],
                           [-0.61768104,  0.6177704 , -0.46894849],
                           [ 0.63978979,  0.63987915, -0.4055221 ]])
    assert np.isclose(v, v_expected, atol=1e-3).all() == True

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
        v[v_ind] += pb.environment_exploration(r_i, r_j, theta_j, H_i, T, r_0)
    assert type(v) == np.ndarray
    assert len(v) > 0
    assert (v!=v0).all() == True
    v_expected = np.array([[-0.55936989, -0.55753435, -0.10465241],
                           [ 0.56114261, -0.55930706, -0.03484197],
                           [-0.56114261,  0.55930706,  0.03484197],
                           [ 0.55936989,  0.55753435,  0.10465241]])
    assert np.isclose(v, v_expected, atol=1e-3).all() == True

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
        r[r_ind] += s_i*pb.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 7.43365823,  7.43558313,  7.89405153],
                           [-7.43197369,  7.42160662,  6.96320247],
                           [ 7.41963269, -7.4339736 ,  6.03448034],
                           [-7.42182292, -7.42368326,  5.10817566]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

# 20
def test_alignment_1():
    # alignment with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    v = np.asarray([[1.0, 0, 0],
                    [0, 2.0, 0],
                    [0, 0, 1.0],
                    [0, 1.0, 0]])
    v0 = np.copy(v) # initial state
    dT = 0.1 # time sampling
    for v_ind in range(len(v)):
        v_i = v[v_ind]
        v_j = np.delete(v, np.array([v_ind]), axis=0)
        r[v_ind] += dT*pb.alignment(v_i, v_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 7.93795243,  8.05338447,  8.02357023],
                           [-7.98509288,  7.90703819,  7.01490712],
                           [ 8.02357023, -7.94661553,  5.93795243],
                           [-7.97642977, -8.01380712,  5.02357023]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

# 21
def test_geofencing_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed

    # sphere function: used as region to be filled
    sphere = lambda x: x[0]**2 + x[1]**2 + x[2]**2 - 4.0
    
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r[r_ind] += s_i*pb.geofencing(r_i, sphere)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 7.42264973,  7.42264973,  7.42264973],
                           [-7.39868318,  7.39868318,  6.47384778],
                           [ 7.37530495, -7.37530495,  5.53147871],
                           [-7.35323833, -7.35323833,  4.59577396]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

# 22
def test_repulsion_1():
    # four robots
    r = np.asarray([[1., 1., 0.1],
                [-1., 1., 0.2],
                [1., -1., 0.3],
                [-1., -1., 0.4]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*pb.repulsion(r_i, r_j, 3.0)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 4.02384627,  3.99892448, -0.35108476],
                           [-2.01678048,  3.87769031, -0.03693001],
                           [ 3.25080306, -1.45297702,  0.23275755],
                           [-1.54036428, -1.42306363,  0.46932458]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

# 23
def test_target_1():
    # target with one robot
    r_i = np.asarray([0., 0., 0.])
    r_i0 = np.copy(r_i)
    s_i = 1.0 # linear speed
    T = np.asarray([8., 8., 8.])
    r_i = s_i*pb.target(r_i, T)
    assert type(r_i) == np.ndarray
    assert len(r_i) > 0
    assert (r_i!=r_i0).all() == True
    r_i_expected = np.array([0.57735027, 0.57735027, 0.57735027])
    assert np.isclose(r_i, r_i_expected, atol=1e-3).all() == True

#24
def test_area_coverage_1():
    # four robots
    r = np.asarray([[1., 1., 0.1],
                [-1., 1., 0.2],
                [1., -1., 0.3],
                [-1., -1., 0.4]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    
    # sphere function: used as region to be filled
    sphere = lambda x: x[0]**2 + x[1]**2 + x[2]**2 - 4.0
    
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*pb.area_coverage(r_i, r_j, sphere, 3.0, 3)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 5.09844667,  5.04875776, -0.5310037],
                           [-1.81142177,  5.02637413, -0.1834818],
                           [ 4.26953751, -1.04896699,  0.1196037],
                           [-1.10401566, -1.05120434,  0.3882858]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

#25
def test_collective_navigation_1():
    # four robots
    r = np.asarray([[1., 1., 0.1],
                [-1., 1., 0.2],
                [1., -1., 0.3],
                [-1., -1., 0.4]])
    r0 = np.copy(r) # initial state
    s_i = 1.0 # linear speed
    T = np.asarray([8., 8., 8.])
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*pb.collective_navigation(r_i, r_j, T, 2.0, 2)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 2.89662258,  2.88554623,  0.52326905],
                           [-0.89195081,  2.75258595,  0.63265339],
                           [ 2.51257400, -0.73771030,  0.78108547],
                           [-0.81041961, -0.79175621,  0.85794976]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True

#26
def test_flocking_1():
    # alignment with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    r0 = np.copy(r) # initial state
    v = np.asarray([[1.0, 0, 0],
                    [0, 2.0, 0],
                    [0, 0, 1.0],
                    [0, 1.0, 0]])
    v0 = np.copy(v) # initial state
    dT = 0.1 # time sampling
    for v_ind in range(len(v)):
        r_i = r[v_ind]
        r_j = np.delete(r, np.array([v_ind]), axis=0)
        v_i = v[v_ind]
        v_j = np.delete(v, np.array([v_ind]), axis=0)
        r[v_ind] += dT*pb.flocking(r_i, r_j, v_i, v_j, 2.0, 2)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
    r_expected = np.array([[ 7.88340982,  7.99900754,  8.0133642 ],
                           [-7.93040664,  7.85249043,  7.01154799],
                           [ 7.96858094, -7.89213842,  5.94140328],
                           [-7.92183938, -7.9591759 ,  5.03375248]])
    assert np.isclose(r, r_expected, atol=1e-3).all() == True
