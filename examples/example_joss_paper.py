#! /usr/bin/env python3

'''
@author: Emerson Andrade
LOC/COPPE/UFRJ      2023
'''

# importing pyswarming behaviors
import pyswarming.behaviors as pb

# importing numpy to work with arrays
import numpy as np

# importing matplotlib to plot the animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Swarm_paper:

    def __init__(self):
        # define each robot (x, y, z) position
        self.robot_positions = np.asarray([[10., 10., 0.],
                                        [-10., 10., 0.],
                                        [-10., -10., 0.],
                                        [10., -10., 0.]])

        # define each robot (roll, pitch, yaw) orientation
        self.robot_orientations = np.asarray([[0., 0., np.pi/4],
                                            [0., 0., 1*0.5*np.pi+(np.pi/4)],
                                            [0., 0., 2*0.5*np.pi+(np.pi/4)],
                                            [0., 0., 3*0.5*np.pi+(np.pi/4)]])

        # define the path lenght of the robots
        self.path_lenghts = 30


        self.position_history = np.asarray([[[self.robot_positions[0][0] for i in range(self.path_lenghts)], [self.robot_positions[0][1] for i in range(self.path_lenghts)]],
                                            [[self.robot_positions[1][0] for i in range(self.path_lenghts)], [self.robot_positions[1][1] for i in range(self.path_lenghts)]],
                                            [[self.robot_positions[2][0] for i in range(self.path_lenghts)], [self.robot_positions[2][1] for i in range(self.path_lenghts)]],
                                            [[self.robot_positions[3][0] for i in range(self.path_lenghts)], [self.robot_positions[3][1] for i in range(self.path_lenghts)]]])

        # set the robot speeds
        self.robot_linear_speed = 0.30
        self.robot_angular_speed = 0.02

        # First set up the figure, the self.axis, and the plot element we want to animate
        self.fig, self.ax = plt.subplots()

    # function to map the orientation angle
    def range_angle(self, angle):
        angle = np.degrees(angle)
        # reduce the angle
        angle =  angle % 360
        # force it to be the positive remainder, so that 0 <= angle < 360  
        angle = (angle + 360) % 360
        return angle

    # animation function. This is called sequentially
    def _animate(self, i):

        self.ax.cla()

        self.ax.set_xlim([-15,15])
        self.ax.set_ylim([-15,15])
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.ax.grid()
        self.ax.set_aspect('equal')
        self.ax.set_title('Aggregation + Heading Consensus + Repulsion behaviors', fontsize=9)

        # path history
        self.ax.plot(self.position_history[0][0], self.position_history[0][1], marker='.', lw=0, c='lightgray')
        self.ax.plot(self.position_history[1][0], self.position_history[1][1], marker='.', lw=0, c='lightgray')
        self.ax.plot(self.position_history[2][0], self.position_history[2][1], marker='.', lw=0, c='lightgray')
        self.ax.plot(self.position_history[3][0], self.position_history[3][1], marker='.', lw=0, c='lightgray')

        # position now
        self.ax.plot(self.robot_positions[0][0], self.robot_positions[0][1], marker='o', lw=0)
        self.ax.plot(self.robot_positions[1][0], self.robot_positions[1][1], marker='o', lw=0)
        self.ax.plot(self.robot_positions[2][0], self.robot_positions[2][1], marker='o', lw=0)
        self.ax.plot(self.robot_positions[3][0], self.robot_positions[3][1], marker='o', lw=0)

        arrow_len = 2.0
        self.ax.arrow(self.robot_positions[0][0], self.robot_positions[0][1], arrow_len*np.cos(self.robot_orientations[0][2]), arrow_len*np.sin(self.robot_orientations[0][2]), head_width=0.45, head_length=0.55, fc='k')
        self.ax.arrow(self.robot_positions[1][0], self.robot_positions[1][1], arrow_len*np.cos(self.robot_orientations[1][2]), arrow_len*np.sin(self.robot_orientations[1][2]), head_width=0.45, head_length=0.55, fc='k')
        self.ax.arrow(self.robot_positions[2][0], self.robot_positions[2][1], arrow_len*np.cos(self.robot_orientations[2][2]), arrow_len*np.sin(self.robot_orientations[2][2]), head_width=0.45, head_length=0.55, fc='k')
        self.ax.arrow(self.robot_positions[3][0], self.robot_positions[3][1], arrow_len*np.cos(self.robot_orientations[3][2]), arrow_len*np.sin(self.robot_orientations[3][2]), head_width=0.45, head_length=0.55, fc='k')

        self.ax.text(0.7, -12.95, 'iteration: %d' % (i), color='red')

        self.robot_positions_copy = self.robot_positions.copy()
        self.robot_orientations_copy = self.robot_orientations.copy()
        
        for r_ind in range(len(self.robot_positions)):
            r_i = self.robot_positions[r_ind]
            r_j = np.delete(self.robot_positions, np.array([r_ind]), axis=0)
            self.robot_positions_copy[r_ind] += self.robot_linear_speed*((pb.aggregation(r_i, r_j)) + pb.repulsion(r_i, r_j, 3.0))

            theta_i = self.robot_orientations[r_ind]
            theta_j = np.delete(self.robot_orientations, np.array([r_ind]), axis=0)
            e = self.range_angle(self.robot_orientations[r_ind]) - self.range_angle(pb.heading_consensus(theta_i, theta_j))

            self.robot_orientations_copy[r_ind][2] += self.robot_angular_speed*np.radians(e[2])

        self.robot_positions = self.robot_positions_copy
        self.robot_orientations = self.robot_orientations_copy

        a00, a01 = self.position_history[0][0].tolist(), self.position_history[0][1].tolist()
        b00, b01 = self.position_history[1][0].tolist(), self.position_history[1][1].tolist()
        c00, c01 = self.position_history[2][0].tolist(), self.position_history[2][1].tolist()
        d00, d01 = self.position_history[3][0].tolist(), self.position_history[3][1].tolist()
        
        a00.append(self.robot_positions[0][0]), a01.append(self.robot_positions[0][1])
        b00.append(self.robot_positions[1][0]), b01.append(self.robot_positions[1][1])
        c00.append(self.robot_positions[2][0]), c01.append(self.robot_positions[2][1])
        d00.append(self.robot_positions[3][0]), d01.append(self.robot_positions[3][1])
        
        self.position_history[0][0], self.position_history[0][1] = a00[1:], a01[1:]
        self.position_history[1][0], self.position_history[1][1] = b00[1:], b01[1:]
        self.position_history[2][0], self.position_history[2][1] = c00[1:], c01[1:]
        self.position_history[3][0], self.position_history[3][1] = d00[1:], d01[1:]

    def simulate(self):
        # call the animator.
        anim = animation.FuncAnimation(self.fig,
                                        self._animate,
                                        frames=720,
                                        interval=1,
                                        blit=False)

        plt.show()

my_swarm = Swarm_paper()
my_swarm.simulate()
