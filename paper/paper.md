---
title: 'PySwarming: a research toolkit for Swarm Robotics'
tags:
  - Python
  - Robotics
  - Swarm
  - Self-Organizing
  - Multi-Robot Systems
authors:
  - name: Emerson Martins de Andrade
    orcid: 0000-0002-5023-8733
    affiliation: "1, 2"
  - name: Antonio Carlos Fernandes
    orcid: 0000-0001-6578-1985
    affiliation: "1, 2"
  - name: Joel Sena Sales Junior
    orcid: 0000-0003-4563-1538
    affiliation: "1, 2"
affiliations:
 - name: Federal University of Rio de Janeiro, Rio de Janeiro, Brazil
   index: 1
 - name: Ocean Engineering Program, Laboratory of Waves and Current, LOC/COPPE/UFRJ, Rio de Janeiro, Brazil
   index: 2
date: 21 July 2023
bibliography: paper.bib

---


# Summary

When considering a system composed by a group of robots, swarm robotics is an approach that can be used to coordinating this group. These swarm robotic systems can be inspired or not by social insects or other animal societies [@trianni:2008], where basic behaviors are usually used to compose complex tasks. These previous mentioned basic behaviors have been studied for a long time, with applications, for example, to flocks, herds, and schools [@reynolds:1987] and multi-robot teams [@balch:1998].
However, software standardization is urgently needed to maximize the applicability of swarm robotics to real-world applications [@nedjah:2019]. Here we introduce PySwarming, a tool that makes easy the coordination of swarms and serves as a centerpiece, organizing different methods developed in the swarm robotics field. Its flexibility (written in Python) and customizability (easily customized by users) encourage interaction and scientific progress in the research community.

# Introduction

Controlling a system composed of a group of robots can be a challenge, then, swarm robotics is an approach that is widely used to coordinate this kind of system. Furthermore, the changes that the field of swarm robotics has experienced in the last decade are unprecedented, with various demonstrations showing the potential of this technology [@drigo:2021]. Moreover, it is important to note that swarm robotics is a subfield of multi-robot systems, itself a subfield of mobile robot research [@dias:2021].
Additionally, these swarming robotic systems may or may not be inspired by social insects and other animal societies [@trianni:2008], where basic behaviors are typically used to compose complex tasks. The aforementioned behavior has long been studied and applied, for instance, to herds, flocks, schools, etc. ([@reynolds:1987], [@toner:1998]), self-driven particles [@vicsek:1995], large collections of robots ([@reif:1999], [@spears:1999]), and the multi-robot teams [@balch:1998].

## Statement of Need

Although, as stated by [@nedjah:2019], a significant issue in the swarm robotics field is the lack of common generic hardware platforms and controller design methodologies. Also, the authors claim an urgent need for standardization of software, to allow a possible flourishing of swarm robotics applicability to real-world applications.

## Related Software Packages

There are various software packages which concerns swarm robotics, [@calderon:2022] presents a good review regarding swarm robotics simulators, platforms and applications. Therefore, for software packages we have, for instance: (1) Buzz, which is a programming language for heterogeneous robot swarms [@pinciroli:2015]. It offers primitives to define swarm behaviors and also single-robot instructions; (2) ChoiRbot by [@testa:2021], which is a toolbox for distributed cooperative robotics based on the Robot Operating System (ROS) 2; (3) ROS2SWARM, which is a package for applications of swarm robotics that provides a library of ready-to-use swarm behavioral primitives [@kaiser:2022]; (4) ARGoS, which is a multi-physics robot simulator, able to simulating large-scale swarms of robots [@pinciroli:2011]; (5) Stage Simulator, which provides a virtual world populated by mobile robots and sensors, along with various objects for the robots to sense and manipulate [@vaughan:2008]; (6) USARSim is a free 3D simulator similar to Gazebo [@carpin:2007]; (7) Swarm-bots [@mondada:2004] is also a robotic simulator, but it is not publicly available; (8) and TeamBots is a Java-based collection of application programs and Java packages for multiagent mobile robotics research [@balch:1998teambots].

# PySwarming
Considering the challenge of organizing the various methods developed in the swarm robotics field, PySwarming comes as a focal point, being flexible (written in Python) and customizable (can be easily adapted by the user), increasing the interaction of the researcher community and the advance of science. Also, PySwarming's characteristic is to make the implementations easy to read, keeping the syntax simple and closer to their sources. For example, the target algorithm by [@zoss:2018] is easily comparable with the mathematical formula of their article.

PySwarming differs from Buzz and TeamBots mainly by the fact that it focuses on swarm behaviors and it is written in Python, which has a thriving ecosystem of third-party libraries. Also, implementations like ChoiRbot and ROS2SWARM require ROS 2 to run, which makes PySwarming more suitable for obtaining swarm behaviors through different platforms other than ROS. Lastly, unlike ARGoS, Stage, USARSim, and Swarm-bots, the main goal of PySwaming is not to be a simulator itself, but a common place for different swarm coordination methods and other tools.

Concerning the algorithms, PySwarming contains implementations from different authors, for instance, Leaderless Coordination [@vicsek:1995], Preferred Direction [@couzin:2005], Aggregation [@zoss:2018], and so on. Also, these algorithms are based on different design methods [@brambilla:2013], for instance, a behavior-based design like the attraction-repulsion algorithm (for details see [@spears:2004]) can drive the robotic swarming by means of virtual forces, where the achieved configuration relies on minimizing the system’s potential energy.
More explanations regarding the algorithms and their use can be found in the API and PySwarming documentation. Also, an example usage (aggregation + heading consensus) is described below.

## Algorithms covered

This library includes the following algorithms to be used in swarm robotics:

* **Leaderless heading consensus**: the collective performs heading consensus [@vicsek:1995];
* **Inverse power**: ajustable attraction and repulsion laws [@reif:1999];
* **Spring**: allows the robots to maintain a desired distance between them [@reif:1999];
* **Force law**: mimics the gravitational force [@spears:1999];
* **Repulsive force**: makes the individuals repulse each other [@helbing:2000];
* **Body force**: introduces a body force that considers the radii of the robots [@helbing:2000];
* **Inter robot spacing**: allows the robots to maintain a desired distance between them [@leonard:2001];
* **Dissipative**: a dissipative force that reduces the "energy" of the robots [@leonard:2001];
* **Leader Following**: the collective performs heading consensus with a leader [@jadbabaie:2003];
* **Collision Avoidance**: the robot stays away from neighbors in the vicinity [@couzin:2005];
* **Attraction and Alignment**: the robot becomes attracted and aligned [@couzin:2005];
* **Preferred Direction**: the robot has a preference to move toward a preset direction [@couzin:2005];
* **Lennard-Jones**: allows the formation of lattices [@pinciroli:2008];
* **Virtual viscosity**: a viscous force that reduces the "oscillation" of the robots [@pinciroli:2008];
* **Modified Attraction and Alignment**: the robot becomes attracted and aligned by considering a “social importance” factor [@freeman:2009];
* **Heading Consensus**: the collective performs heading consensus [@chamanbaz:2017];
* **Perimeter Defense**: the robots maximize the perimeter covered in an unknown environment [@chamanbaz:2017];
* **Environment exploration**: provides spatial coverage [@chamanbaz:2017];
* **Aggregation**: makes all the individuals aggregate collectively [@zoss:2018];
* **Alignment**: the collective performs heading consensus [@zoss:2018];
* **Geofencing**: attract the robots towards area A [@zoss:2018];
* **Repulsion**: makes all the individuals repulse collectively [@zoss:2018];
* **Target**: the robot goes to an specific target location [@zoss:2018];
* **Area coverage**: using the Geofencing and Repulsion algorithms [@zoss:2018];
* **Collective navigation**: using the Target and Repulsion algorithms [@zoss:2018];
* **Flocking**: using the Aggregation, Repulsion and Alignment algorithms [@zoss:2018];

## Example Usage 
To start our example, we will define a set of four robots assuming we have access to their positions and orientations. Initially, they are positioned far from each other, and also they have different orientations, as can be observed in \autoref{fig1}.

![The initial state of the four robots. Each colored circle is a robot and the arrows are indicating their orientation. The iteration number is the red text.\label{fig1}](fig1.png)

Then, using `PySwarming` we iterate over time by summing two different behaviors: (1) Aggregation and (2) Heading Consensus. Each of these behaviors is applied to each robot at each timestep. As expected, the robots will aggregate and adjust their headings simultaneously over the simulation. The mid and final results are shown in \autoref{fig2}.

![Mid (left) and final (right) state of the four robots. The gray path is the plot of the last 30 iterations of each robot.\label{fig2}](fig2.png)

The above simulation can be done by using other `PySwarming` behaviors, such as Attraction and Alignment, with just a few lines of code, which demonstrates the simplicity of `PySwarming`. Finally, the code for this simulation can be found in our [examples](https://github.com/mrsonandrade/pyswarming/tree/main/examples) directory.

# Conclusion

Finally, we identified that despite the existence of many other software packages, PySwarming comes as a focal point, filling an important space, which regards the organization of the different methods developed over the last decades. Also, the fact that the library is written in Python confers flexibility and customization, increasing the interaction of the researcher community and the advancement of science.
 

# Acknowledgements
The authors would like to thank the Human Resources Program from the National Agency of Oil, Gas and Bio Combustibles – PRH-ANP for the financial support.
This work was supported by "Coordenação de Aperfeiçoamento de Pessoal de Nível Superior - Brasil (CAPES)", [LOC/COPPE/UFRJ](https://www.loc.ufrj.br/index.php/en/) ([Laboratory of Waves and Current](https://www.loc.ufrj.br/index.php/en/) - [Federal University of Rio de Janeiro](https://ufrj.br/en/)) and the National Council for Scientific and Technological Development (CNPq), which are gratefully acknowledged.

# References



