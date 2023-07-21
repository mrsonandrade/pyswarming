#! /usr/bin/env python3

'''
@author: Emerson Andrade
LOC/COPPE/UFRJ      2023
'''

# importing pyswarming behaviors
import pyswarming.behaviors as ps

# importing numpy to work with arrays
import numpy as np

# importing matplotlib to plot the animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# importing functools.partial to use in the animation
from functools import partial

# define each robot (x, y, z) position
robot_poses = np.asarray([[1., 1., 0.],
                [-1., 1., 0.],
                [1., -1., 0.],
                [-1., -1., 0.]])

# set the robot linear velocity
robot_speed = 0.10

# define a target (x, y, z) position
swarm_target = np.asarray([35., 35., 0.])

# First set up the figure, the axis, and the plot element we want to animate
fig, ax = plt.subplots()

ax.set_xlim([-10,50])
ax.set_ylim([-10,50])
ax.set_xlabel('X(m)')
ax.set_ylabel('Y(m)')
ax.grid()
ax.set_aspect('equal')
ax.set_title('Collective Navigation')

robot1, = ax.plot([], [], marker='o', lw=0)
robot2, = ax.plot([], [], marker='o', lw=0)
robot3, = ax.plot([], [], marker='o', lw=0)
robot4, = ax.plot([], [], marker='o', lw=0)

target = ax.plot(swarm_target[0], swarm_target[1], marker='x', color='red', lw=0)

# initialization function: plot the background of each frame
def init():
    robot1.set_data([], [])
    robot2.set_data([], [])
    robot3.set_data([], [])
    robot4.set_data([], [])
    return (robot1,robot2,robot3,robot4,)

# animation function. This is called sequentially
def animate(i, robot_poses):
    for r_ind in range(len(robot_poses)):
        r_i = robot_poses[r_ind]
        r_j = np.delete(robot_poses, np.array([r_ind]), axis=0)
        robot_poses[r_ind] += robot_speed*ps.collective_navigation(r_i, r_j, swarm_target, 5.0)
    robot1.set_data(robot_poses[0][0], robot_poses[0][1])
    robot2.set_data(robot_poses[1][0], robot_poses[1][1])
    robot3.set_data(robot_poses[2][0], robot_poses[2][1])
    robot4.set_data(robot_poses[3][0], robot_poses[3][1])
    return (robot1,robot2,robot3,robot4,)

# call the animator. blit=True means only re-draw the parts that 
# have changed.
anim = animation.FuncAnimation(fig, partial(animate, robot_poses=robot_poses), init_func=init,
                               frames=960, interval=1, blit=True)

anim
plt.show()