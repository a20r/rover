Rover
=====

A Planner for Autonomuos Risk-Sensitive Coverage by a Team of Unmanned Aerial Vehicles

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
