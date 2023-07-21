import pyswarming.swarm as ps

import numpy as np

def test_swarm():
    #@pytest.fixture(scope='function')
    my_swarm = ps.Swarm(n = 10, # number of robots
                        linear_speed = 0.5, # linear speed of each robot
                        dT = 1.0, # sampling time
                        deployment_point_limits = [[0.0, 0.0, 0.0], [5.0, 5.0, 0.0]], # lower and upper limits for the position deployment
                        deployment_orientation_limits = [[0.0, 0.0, 0.0], [0.0, 0.0, 2*3.1415]], # lower and upper limits for the orientation deployment
                        distribution_type =  'uniform', # type of distribution used to deploy the robots
                        plot_limits = [[-50.0, 50.0], [-50.0, 50.0]], # plot limits x_lim, y_lim
                        behaviors = ['target','aggregation']) # list of behaviors
    my_swarm.behaviors_dict['r_out']['target']['T'] = [-40, -40, 0] # setting the target position [x, y, z]
    
    assert type(my_swarm.pose) == np.ndarray # type
    assert my_swarm.pose.shape == (10, 6) # shape
    assert (my_swarm.pose[:,:3] >= 0.0).all() # coordinates
    assert (my_swarm.pose[:,:3] <= 5.0).all() # coordinates
    assert (my_swarm.pose[:,3:] >= 0.0).all() # orientations
    assert (my_swarm.pose[:,3:] <= 2*3.1415).all() # orientations
    assert my_swarm.behaviors == ['target','aggregation'] # behaviors

