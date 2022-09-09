# pyswarming

<img align="left" src="docs/readme_pics/logo.png">

`pyswarming` is a research toolkit for Swarm Robotics.


## Installation
You can install ``pyswarming`` from PyPI using pip (**Recommended**):
```
pip install pyswarming
```

## Dependencies

`pyswarming`'s dependencies are: `numpy`.


## Documentation
The official documentation is hosted on **[ReadTheDocs](https://pyswarming.readthedocs.io)**.


## Examples
Considering a swarm of robots, they can show different behaviors by using ``pyswarming``.
```python
# importing the swarming behaviors
import pyswarming.behaviors as ps

# importing numpy to work with arrays
import numpy as np
```


### Target 
To simplify, considering just one robot, the target behavior is shown below.
```python
# define the robot (x, y, z) position
r_i = np.asarray([0., 0., 0.])

# set the robot linear velocity
s_i = 1.0

# define a target (x, y, z) position
T = np.asarray([8., 8., 0.])

for t in range(15):

    # print the robot (x, y, z) position
    print(r_i)

    # update the robot (x, y, z) position
    r_i += s_i*ps.target(r_i, T)
```
![Target](Examples/pics/Target.gif)

## Contributing to pyswarming
All kind of contributions are welcome: 
* Improvement of code with new features, bug fixes, and  bug reports
* Improvement of documentation
* Additional tests

Follow the instructions [here](https://pyswarming.readthedocs.io/en/latest/Contribution.html)
for submitting a PR.

If you have any ideas or questions, feel free to open an issue.


## Acknowledgements
This research is supported by CAPES (Coordination of Improvement of Higher Education Personnel), [LOC/COPPE/UFRJ](https://www.loc.ufrj.br/index.php/en/) ([Laboratory of Waves and Current](https://www.loc.ufrj.br/index.php/en/) - [Federal University of Rio de Janeiro](https://ufrj.br/en/)) and CNPq (Brazilian National Council for Scientific and Technological Development), which are gratefully acknowledged.
