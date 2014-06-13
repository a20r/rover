
Related Work Survey
-------------------

### 3D Path Planning for UAVs for Maximum Information Collection 

- URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6564676

- Plans the paths of UAVs in 3 dimensions that avoids forbidden regions and
  maximizes information collection from desired regions

- Formulates problem as a multiple Travelling Salesman Problem and uses the
  Pattern Search method to solve this problem.

- Using the grid of desired and forbidden regions, this approach uses a genetic
  algorithm with initial population of the solutions from the mTSP to optimize
  the waypoints being used for the UAVs.

- They don't talk about scalability, they only did tests with up to 3 UAVs, and
  they are using a GA whilst solving mTSP. It is probably safe to say that
  their algorithm doesn't scale well with respect to anything in their
  parameter space.


### Path Planning of Autonomous Underwater Vehicles for Adaptive Sampling Using Mixed Integer Linear Programming

- URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4768634

- Path planning algorithm that optimizes an objective function to move around
  the ocean to collect information about desired regions

- It is assumed that the desired regions are known *a priori*

- The objective is to sample the regions of greatest uncertainty and to
  maximize the information gain

- It plans using a constraint programming based on motion constraints

- I have no idea what their results are supposed to mean

### Physics-Inspired Robotic Motion Planning for Cooperative Bayesian Target Detection

- Uses a states of matter approach for area coverage and target detection /
  tracking.

- At gas state, the quads move down a surface gradient w/ random walk

- At liquid state, quads move down a potential surface

- At solid state, quads move down a potential gradient and have a spring force
  between other quads

- The surface simulates the temperature and is governed by an inverse log
  likelihood ratio w/ temperature diffusion.

- When target is "seen", the area within a certain radius of the target on the
  surface gets its value decremented and the heat is diffused.

- Has inherent potential field issues (i.e. valleys, riveras, local minima)

- Non simulation testing has only been performed with one quadrotor.

- Does NOT perform well for coverage, but does very well for tracking.

