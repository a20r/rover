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
    
## Configuration
This project relies on a JSON configuration file. It indicates how the simulation is to be run.
Below is a schema for the configuration file to run the simulation.

```
{
    "width": <Width of scene>,
    "height": <Height of scene>,
    "num_quads": <Number of quadrotors in the swarm>,
    "min_height": <Minimum Altitude>,
    "max_height": <Maximum Altitude>,
    "step_size": <Maximum speed>,
    "viewing_angle": <Field of view of the sensor>,
    "num_steps": <Number of iterations to run>,
    "sq_height": <The altitude that the quadrotors should try to stay>,
    "sq_std": <Scaling constant for penalties in the deviation from 
        the prefered altitude>,
    "risk_constant": <Scaling constant for pentalties in the risk regression>,
    "camera_angle_freedom": <How much the sensor can move>,
    "initial_camera_angle": <Angle of the sensor relative to the vertical>,
    "orientation_freedom": <The freedom of change in yaw>,
    "show_time_grid": <Boolean: Whether the time grid should be shown or not>,
    "practical": <Boolean: Whether or not the experiment will be run 
        on actual quads>,
    "planner": <Algorithm name to run. [USE "rover_gaussian"]>,

    [IF "practical"]
    "names": {
        <Name of robot as stored in ZeroMQ-ROS database>: 
            <Topic to send velocities to. [USE "cmd_vel"]>,
        ...
    }
    [END IF]

    "min_safe_distance": <Minimum safe distance between quadrotors>,
    "scene": <Scene to be used for the risk. Samples are shown in `scenes/`>,

    "img_file": <Where the risk heatmap is to be saved>,
    "out_file": <Where the output statistics are to be saved>,
    "verification_file": <Where the trajectories are to be saved>
}
```
