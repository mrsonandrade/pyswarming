import pyswarming.behaviors as ps
import numpy as np


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

def test_aggregation_repulsion_1():
    # aggregation plus repulsion with four robots
    r = np.asarray([[8., 8., 8.],
                    [-8., 8., 7.],
                    [8., -8., 6.],
                    [-8., -8., 5.]])
    r0 = np.copy(r)
    s_i = 1.0
    for r_ind in range(len(r)):
        r_i = r[r_ind]
        r_j = np.delete(r, np.array([r_ind]), axis=0)
        r[r_ind] += s_i*(ps.aggregation(r_i, r_j) + ps.repulsion(r_i, r_j, 5.0))
    assert type(r) == np.ndarray
    assert len(r) > 0
    assert (r!=r0).all() == True
