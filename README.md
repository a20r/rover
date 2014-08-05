Rover
=====

A Planner for Autonomuos Risk-Sensitive Coverage by a Team of Unmanned Aerial Vehicles

## Overview
This project proposes a path-planning approach
to enable a team of unmanned aerial vehicles (UAVs) to
efficiently conduct surveillance of sensitive areas. The proposed
approach, termed Rover, seeks to maximize the area covered by the
sensors mounted on each UAV while maintaining high sensor
data quality and minimizing detection risk. Rover uses a
dynamic grid to keep track of the parts of the space that have
been surveyed and the times that they were last surveyed. This
information is then used to move the UAVs toward areas that
have not been covered in a long time. Moreover, a nonlinear
optimization formulation is used to determine the altitude at
which each UAV flies. The efficiency and scalability of Rover
is demonstrated in simulation using complex environments
and an increasing number of UAVs to conduct risk-sensitive
surveillance.

## Running
This is academic code so to all you need to do is run it and watch. A few things need
to happen in order for you to run the code.

    $ # Runs Rviz in background to view simulation
    $ rosrun rviz rviz &
    $ # Sends a static transform in order for visualization markers to be viewed
    $ roslaunch configs/tf.launch &
    
These programs need to be running in order to run the simulation. However, you do not
need to run them each time you want to run the simulation. Just keep them in the background.
To run the simulation, use the command below.

    $ # Runs the simulation
    $ python run.py [configuration file name]
