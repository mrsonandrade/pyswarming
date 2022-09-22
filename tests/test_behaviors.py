import pyswarming.behaviors as ps
import numpy as np


def test_leaderless_coordination_1():
    # leaderless coordination with four robots
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta)
    for theta_ind in range(len(theta)):
        theta_i = theta[theta_ind]
        theta_j = np.delete(theta, np.array([theta_ind]), axis=0)
        theta[theta_ind] = ps.leaderless_coordination(theta_i, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).any() == True


def test_leader_following_1():
    # leader following with four robots
    theta_0 = np.asarray([0.78, 0.78, 0.78])
    b_i = 1
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta)
    for theta_ind in range(len(theta)):
        theta_i = theta[theta_ind]
        theta_j = np.delete(theta, np.array([theta_ind]), axis=0)
        theta[theta_ind] = ps.leader_following(theta_i, theta_j, theta_0, b_i)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True


def test_collision_avoidance_1():
    # collision avoidance with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.collision_avoidance(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True


def test_attraction_alignment_1():
    # attraction and alignment with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta)
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        theta_j = np.delete(theta, np.array([r_ind]), axis=0)
        theta[r_ind] = ps.attraction_alignment(r_i, r_j, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True


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


def test_modified_attraction_alignment_1():
    # modified attraction and alignment with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta)
    h_j = np.asarray([1.0, 1.0, 1.0, 1.0])
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        theta_j = np.delete(theta, np.array([r_ind]), axis=0)
        theta[r_ind] = ps.modified_attraction_alignment(r_i, r_j, theta_j, h_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).all() == True


def test_heading_consensus_1():
    # heading consensus with four robots
    theta = np.asarray([[0.78, 0.78, 0.78],
                        [-0.78, 0.78, 0.78],
                        [0.78, -0.78, 0.78],
                        [-0.78, -0.78, 0.78]])
    theta0 = np.copy(theta)
    for theta_ind in range(len(theta)):
        theta_i = theta[theta_ind]
        theta_j = np.delete(theta, np.array([theta_ind]), axis=0)
        theta[theta_ind] = ps.heading_consensus(theta_i, theta_j)
    assert type(theta) == np.ndarray
    assert len(theta) > 0
    assert (theta!=theta0).any() == True


def test_perimeter_defense_1():
    # perimeter defense with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.perimeter_defense(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True


def test_aggregation_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True


def test_alignment_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True


def test_geofencing_1():
    # aggregation with four robots
    r = np.asarray([[8., 8., 8.],
                [-8., 8., 7.],
                [8., -8., 6.],
                [-8., -8., 5.]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.aggregation(r_i, r_j)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True


def test_repulsion_1():
    # repulsion with four robots
    r = np.asarray([[1., 1., 0.1],
                [-1., 1., 0.2],
                [1., -1., 0.3],
                [-1., -1., 0.4]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*ps.repulsion(r_i, r_j, 3.0)
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True


def test_target_1():
    # target with one robot
    r_i = np.asarray([0., 0., 0.])
    r_i0 = np.copy(r_i)
    s_i = 1.0
    T = np.asarray([8., 8., 8.])
    r_i = s_i*ps.target(r_i, T)
    assert type(r_i) == np.ndarray
    assert len(r_i) > 0
    assert (r_i!=r_i0).all() == True

