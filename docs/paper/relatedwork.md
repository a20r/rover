
Related Work Survey
-------------------

# 3D Path Planning for UAVs for Maximum Information Collection 

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


