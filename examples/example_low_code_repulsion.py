#! /usr/bin/env python3

'''
@author: Emerson Andrade
LOC/COPPE/UFRJ      2023
'''

# importing the swarm creator
import pyswarming.swarm as ps

# creating the swarm
my_swarm = ps.Swarm(n = 10, # number of robots
                    linear_speed = 0.5, # linear speed of each robot
                    dT = 1.0, # sampling time
                    deployment_point_limits = [[0.0, 0.0, 0.0], [5.0, 5.0, 0.0]], # lower and upper limits for the position deployment
                    deployment_orientation_limits = [[0.0, 0.0, 0.0], [0.0, 0.0, 2*3.1415]], # lower and upper limits for the orientation deployment
                    distribution_type =  'uniform', # type of distribution used to deploy the robots
                    plot_limits = [[-50.0, 50.0], [-50.0, 50.0]], # plot limits x_lim, y_lim
                    behaviors = ['repulsion']) # list of behaviors
my_swarm.simulate()