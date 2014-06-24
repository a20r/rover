
Related Work Survey
-------------------

### 3D Path Planning for UAVs for Maximum Information Collection 

- **URL**: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6564676

- **BibTeX**: Ergezer14 

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

- **URL**: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4768634

- **BibTeX**: Yilmaz08 

- Path planning algorithm that optimizes an objective function to move around
  the ocean to collect information about desired regions

- It is assumed that the desired regions are known *a priori*

- The objective is to sample the regions of greatest uncertainty and to
  maximize the information gain

- It plans using a constraint programming based on motion constraints

- I have no idea what their results are supposed to mean

### Physics-Inspired Robotic Motion Planning for Cooperative Bayesian Target Detection

- **In press. N. Sydney et al. 2014**

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

### Physics-Aware Informative Coverage Planning for Autonomous Vehicles

- **In press. M. Kuhlman et al. ICRA 2014.**

- Tries to solve the problem of informed single vehicle persistent monitoring
  of a given area.

- Given a closed set of way points, *t*, update the waypoints in *t* to
  minimize path cost and maximize the information gained.

- Uses a Markov Decision Process (MDP) to generate collision free way points.
  These MDPs take into account the uncertainty of being blown off course.

- The results show that the collision free paths are developed almost all of
  the time (> 88%). There is no mention about coverage or about how the planner
  optimizes the objective function.

- This algorithm is for static desired regions and static obstacles so the path
  is pre-generated. The generation takes a few minutes to make the path and is
  therefore not scalable and cannot be used in a real-time situation.

- Lastly, the approach is only feasible for a single vehicle, for multiple
  vehicles the solution would scale exponentially.

### Application of Grazing-Inspired Guidance Laws to Autonomous Information Gathering

- **In press. T. Apker et al. Accepted to IROS 2014**

- Bio-inspired algorithm that uses simple laws derived from animal grazing
  behaviour to guide a swarm of robots to cooperatively gather information
  about the environment.

- Uses fixed altitude quadrotors as the agent. Does not account for regions of
  risk but does encode desired regions in the evidence grid used as the "food"
  for grazing

- Results have shown that the grazing behaviour converges more quickly to total
  information coverage than traditional lawnmower approaches. 

- Paper seems to care more about the practicality of the sensor fusion,
  bandwidth, and control than the theoretic path planning

- The work in its current form seems to only be concerned with static
  information surfaces. 

### Path planning for data assimilation in mobile environmental monitoring systems

- **BibTeX**: Hover09

- A path planning algorithm for continuous replanning based on predictions made
  by the assimilated data gathered during planner execution.

- Predictive models are made based on the sensor data assimilation. These
  models are used to re-plan the motion of the autonomous aquatic surface
  vehicle.

- The current implementation does not take depth into account and only plans in
  two dimensions.

- The planner does not scale well. The solution works for small to mid-scaled
  problems.
