"""
``pyswarming.swarm``
========================

The PySwarming swarm class allows the creation of virtual swarm to be used with
the behaviors functions, for details, see the documentation available in two forms:
docstrings provided with the code, and a loose standing reference guide, available
from `the PySwarming homepage <https://github.com/mrsonandrade/pyswarming>`_..

Functions present in pyswarming.swarm are listed below.

Swarm
---------

   simulate

"""

__all__ = ['Swarm']

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import behaviors as bh

class Swarm:
    """
    Creates a Swarm object of n robots, which allows the use of
    different behaviors algorithms.

    Parameters
    ----------
    n : int
        integer number of robots, must be greater than 1 (n > 1).

    linear_speed : float
        float number giving the linear speed of each robot. This
        speed is used for those behaviors that have an orientation or
        nonsimensional contributions.

    dT : float
        float number giving the sampling time.

    deployment_point : list
        list containing the swarm deployment point.

    plot_limits : list
        list containing the plot limits (matplotlib).

    behaviors : list
        list containing the behaviors to be simulated.

    Example 1
    --------
    >>> my_swarm = Swarm(10, # number of robots
                 0.5, # linear speed of each robot
                 1.0, # sampling time
                 [0.0, 0.0], # robots deployed randomly around x = 0.0, y = 0.0 (+- 5.0 meters)
                 [[-50.0, 50.0], [-50.0, 50.0]], # plot limits x_lim, y_lim
                 ['collective_navigation']) # list of behaviors
    >>> my_swarm.behaviors_dict['r_out']['collective_navigation']['alpha'] = 2.0  # setting the strength of the repulsion
    >>> my_swarm.behaviors_dict['r_out']['collective_navigation']['T'] = np.array([-40, -40, 0]) # setting the target
    >>> my_swarm.simulate()

    Example 2
    --------
    >>> my_swarm = Swarm(10, # number of robots
                 0.5, # linear speed of each robot
                 1.0, # sampling time
                 [0.0, 0.0], # robots deployed randomly around x = 0.0, y = 0.0 (+- 5.0 meters)
                 [[-50.0, 50.0], [-50.0, 50.0]], # plot limits x_lim, y_lim
                 ['target','aggregation']) # list of behaviors
    >>> my_swarm.behaviors_dict['r_out']['target']['T'] = np.array([-40, -40, 0]) # setting the target
    >>> my_swarm.simulate()
    """

    def __init__(self, n,
                 linear_speed = 0.5,
                 dT = 1.0,
                 deployment_point = [0.0, 0.0],
                 plot_limits = [[-50.0, 50.0], [-50.0, 50.0]],
                 behaviors = ['target']):

        if n <= 1:
            raise Exception("The number of robots must be greater than 1 (n > 1).")

        self.n = n
        self.dimensions = 2 # this version allows the creation of 2D swarms
        self.linear_speed = linear_speed
        self.dT = dT
        self.plot_limits = plot_limits
        self.deployment_point = deployment_point
        self.behaviors = behaviors
        self.pose = self._create_robots()
        self.behaviors_dict = {'r_out':{'aggregation': {'function':None},
                                          'repulsion': {'function':None,
                                                        'alpha': 10.0,
                                                        'd': 2},
                                          'target': {'function':None,
                                                     'T': np.array([30, 30, 30])},
                                          'collective_navigation': {'function':None,
                                                                    'T': np.array([30, 30, 30]),
                                                                    'alpha': 10.0,
                                                                    'd': 2}},
                                 'theta_out':{'leaderless_heading_consensus': {'function':None},
                                              'heading_consensus': {'function':None}}}

    def _create_robots(self):
        """
        Creates an array of n robots with
        user-specified parameters.
        """

        position = np.random.uniform(low=self.deployment_point[0]-5.0, high=self.deployment_point[1]+5.0, size=(self.n, 3, ))
        orientation = np.random.uniform(low=0, high=2*np.pi, size=(self.n, 3, ))
        pose = np.concatenate((position, orientation), axis=1)

        if self.dimensions==2:
            pose[:,2] = 0 # z
            pose[:,3] = 0 # roll
            pose[:,4] = 0 # pitch

        return pose

    # animation function. This is called sequentially
    def _animate(self, i):

        self.ax.cla()

        self.ax.set_xlim(self.plot_limits[0])
        self.ax.set_ylim(self.plot_limits[1])
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.ax.grid()
        
        pose = self.pose.copy()
        r = pose[:,:3]
        theta = pose[:,3:]

        arrow_len = 4.0

        if self.dimensions == 2:
            self.ax.set_aspect('equal')
            for r_ind in range(len(r)):
                self.ax.plot(r[r_ind][0], r[r_ind][1], marker='o', lw=0)
                self.ax.plot([r[r_ind][0], r[r_ind][0]+arrow_len*np.cos(theta[r_ind][2])],
                             [r[r_ind][1], r[r_ind][1]+arrow_len*np.sin(theta[r_ind][2])], color='k')
                
        for r_ind in range(len(r)):
            r_i = r[r_ind]
            r_j = np.delete(r, np.array([r_ind]), axis=0)

            theta_i = theta[r_ind]
            theta_j = np.delete(theta, np.array([r_ind]), axis=0)

            self.behaviors_dict['r_out']['aggregation']['function'] = bh.aggregation(r_i, r_j)
            self.behaviors_dict['r_out']['repulsion']['function'] = bh.repulsion(r_i,
                                                                                   r_j,
                                                                                   self.behaviors_dict['r_out']['repulsion']['alpha'],
                                                                                   self.behaviors_dict['r_out']['repulsion']['d'])
            self.behaviors_dict['r_out']['target']['function'] = bh.target(r_i,
                                                                             self.behaviors_dict['r_out']['target']['T'])
            self.behaviors_dict['r_out']['collective_navigation']['function'] = bh.collective_navigation(r_i,
                                                                                                           r_j,
                                                                                                           self.behaviors_dict['r_out']['collective_navigation']['T'],
                                                                                                           self.behaviors_dict['r_out']['collective_navigation']['alpha'],
                                                                                                           self.behaviors_dict['r_out']['collective_navigation']['d'])

            self.behaviors_dict['theta_out']['leaderless_heading_consensus']['function'] = bh.leaderless_heading_consensus(theta_i, theta_j)
            self.behaviors_dict['theta_out']['heading_consensus']['function'] = bh.heading_consensus(theta_i, theta_j)

            behaviors_output = []
            for behavior_i in self.behaviors:
                try:
                    behaviors_output.append([self.behaviors_dict['r_out'][behavior_i]['function']])
                except:
                    try:
                        behaviors_output.append([self.behaviors_dict['theta_out'][behavior_i]['function']])
                    except:
                        print('behavior not found: '+behavior_i)
            
            if len(behaviors_output)>0:
                # in this code all the behaviors are transformed into a normalized orientation
                r_sum = (np.sum(np.asarray([r_out[0] for r_out in behaviors_output]), axis=0))
                r_normalized = r_sum/np.linalg.norm(r_sum)
                r[r_ind] += r_normalized * self.linear_speed * self.dT
                theta[r_ind][2] = np.arctan2(r[r_ind][1], r[r_ind][0])

        self.pose = np.concatenate((r, theta), axis=1)

    def simulate(self):

        # First set up the figure and the axis
        if self.dimensions == 2:
            self.fig, self.ax = plt.subplots()

        anim = animation.FuncAnimation(self.fig, self._animate, frames=720, interval=1, blit=False)

        plt.show()
