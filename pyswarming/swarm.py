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

    deployment_point_limits : list
        list containing two lists with the swarm deployment coordinate limits,
        where the first list is the lower limit [x_min, y_min, z_min] and the
        second is the upper limit [x_max, y_max, z_max].
        For the 'none' distribution_type only the lower limit is considered;
        For the 'uniform' distribution_type both limits are considered;
        For the 'gaussian' distribution_type the lower limit is considered
        as the mean of the distribution and the upper limit is considered
        as the standard deviation of the distribution.

    deployment_orientation_limits : list
        list containing two lists with the swarm deployment orientation limits,
        where the first list is the lower limit [roll_min, pitch_min, yaw_min]
        and the second is the upper limit [roll_max, pitch_max, yaw_max].
        For the 'none' distribution_type only the lower limit is considered;
        For the 'uniform' distribution_type both limits are considered;
        For the 'gaussian' distribution_type the lower limit is considered
        as the mean of the distribution and the upper limit is considered
        as the standard deviation of the distribution.

    distribution_type : {'none', 'uniform', 'gaussian'}
        type of distribution used to create the robots.
        - 'none' : the creation is not based in any distribution.
        - 'uniform' : the creation is based in an uniform distribution.
        - 'gaussian' : the creation is based in an gaussian distribution.

    plot_limits : list
        list containing the plot limits (matplotlib).

    behaviors : list
        list containing the behaviors to be simulated.
    """

    def __init__(self, n,
                 linear_speed = 0.5,
                 dT = 1.0,
                 deployment_point_limits = [[0.0, 0.0, 0.0], [5.0, 5.0, 0.0]],
                 deployment_orientation_limits = [[0.0, 0.0, 0.0], [0.0, 0.0, 2*np.pi]],
                 distribution_type =  'uniform',
                 plot_limits = [[-50.0, 50.0], [-50.0, 50.0]],
                 behaviors = ['target']):

        if n <= 1:
            raise Exception("The number of robots must be greater than 1 (n > 1).")

        self.n = n
        self.dimensions = 2 # this version allows the creation of 2D swarms
        self.linear_speed = linear_speed
        self.dT = dT
        self.plot_limits = plot_limits
        self.deployment_point_limits = deployment_point_limits
        self.deployment_orientation_limits = deployment_orientation_limits
        self.distribution_type = distribution_type
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

        if self.distribution_type=='none':
            position = np.asarray([self.deployment_point_limits[0] for i in range(self.n)])
            orientation = np.asarray([self.deployment_orientation_limits[0] for i in range(self.n)])

        elif self.distribution_type=='uniform':
            position = np.random.uniform(low=self.deployment_point_limits[0],
                                         high=self.deployment_point_limits[1],
                                         size=(self.n, 3, ))
            orientation = np.random.uniform(low=self.deployment_orientation_limits[0],
                                            high=self.deployment_orientation_limits[1],
                                            size=(self.n, 3, ))
            
        elif self.distribution_type=='gaussian':
            position = np.random.normal(loc=self.deployment_point_limits[0],
                                         scale=self.deployment_point_limits[1],
                                         size=(self.n, 3, ))
            orientation = np.random.normal(loc=self.deployment_orientation_limits[0],
                                            scale=self.deployment_orientation_limits[1],
                                            size=(self.n, 3, ))

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
                                                                           np.asarray(self.behaviors_dict['r_out']['target']['T']))
            self.behaviors_dict['r_out']['collective_navigation']['function'] = bh.collective_navigation(r_i,
                                                                                                           r_j,
                                                                                                           np.asarray(self.behaviors_dict['r_out']['collective_navigation']['T']),
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

    def simulate(self,
                 frames = 720,
                 interval = 1,
                 blit = False,
                 repeat = False,
                 mode = 'pltshow'):

        # First set up the figure and the axis
        if self.dimensions == 2:
            self.fig, self.ax = plt.subplots()

        if mode == 'pltshow':
            anim = animation.FuncAnimation(self.fig, self._animate, frames=frames, interval=interval, blit=blit, repeat=repeat)
            plt.show()

        elif mode == 'anim':
            import warnings
            warnings.filterwarnings("ignore")
            anim = animation.FuncAnimation(self.fig, self._animate, frames=frames, interval=interval, blit=blit, repeat=repeat)
            return anim
        
        elif mode == 'simulate':
            for time_i in range(frames):
                self._animate(time_i)
            return self.pose

        
