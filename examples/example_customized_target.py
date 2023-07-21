#! /usr/bin/env python3

'''
@author: Emerson Andrade
LOC/COPPE/UFRJ      2023
'''

# importing the swarming behaviors
import pyswarming.behaviors as ps

# importing numpy to work with arrays
import numpy as np

# importing matplotlib to plot the animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# importing functools.partial to use in the animation
from functools import partial

# define each robot (x, y, z) position
robot_poses = np.asarray([0., 0., 0.])

# set the robot linear velocity
robot_speed = 0.025

# define a target (x, y, z) position
swarm_target = np.asarray([8., 8., 0.])

# First set up the figure, the axis, and the plot element we want to animate
fig, ax = plt.subplots()

ax.set_xlim([-2,10])
ax.set_ylim([-2,10])
ax.set_xlabel('X(m)')
ax.set_ylabel('Y(m)')
ax.grid()
ax.set_aspect('equal')
ax.set_title('Target behavior')

robot, = ax.plot([], [], marker='o', lw=0)
target = ax.plot(swarm_target[0], swarm_target[1], marker='x', color='red', lw=0)

# initialization function: plot the background of each frame
def init():
    robot.set_data([], [])
    return (robot,)

# animation function. This is called sequentially
def animate(i, robot_poses):
    r_i = robot_poses
    r_i += robot_speed*ps.target(r_i, swarm_target)
    robot.set_data(r_i[0], r_i[1])
    return (robot,)

# call the animator. blit=True means only re-draw the parts that 
# have changed.
anim = animation.FuncAnimation(fig, partial(animate, robot_poses=robot_poses), init_func=init,
                               frames=480, interval=1, blit=True)

anim
plt.show()