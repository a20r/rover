# Configuration
This project relies on a JSON configuration file. It indicates how the simulation is to be run.
Below is a schema for the configuration file to run the simulation. Once you create the configuration
JSON file, you can pass it in as a command line argument to the main program.

```
{
    "width": Width of scene,
    "height": Height of scene,
    "num_quads": Number of quadrotors in the swarm,
    "min_height": Minimum Altitude,
    "max_height": Maximum Altitude,
    "step_size": Maximum speed,
    "viewing_angle": Field of view of the sensor,
    "num_steps": Number of iterations to run,
    "sq_height": The altitude that the quadrotors should try to stay,
    "sq_std": Scaling constant for penalties in the deviation from 
        the prefered altitude,
    "risk_constant": Scaling constant for pentalties in the risk regression,
    "camera_angle_freedom": How much the sensor can move,
    "initial_camera_angle": Angle of the sensor relative to the vertical,
    "orientation_freedom": The freedom of change in yaw
    "show_time_grid": Boolean: Whether the time grid should be shown or not,
    "practical": Boolean: Whether or not the experiment will be run 
        on actual quads,
    "planner": Algorithm name to run. [USE "rover_gaussian"],

    [IF "practical"]
    "names": {
        Name of robot as stored in ZeroMQ-ROS database: 
            Topic to send velocities to. [USE "cmd_vel"],
        ...
    }
    [END IF]

    "min_safe_distance": Minimum safe distance between quadrotors,
    "scene": Scene to be used for the risk. Samples are shown in `scenes/`,

    "img_file": Where the risk heatmap is to be saved,
    "out_file": Where the output statistics are to be saved,
    "verification_file": Where the trajectories are to be saved
}
```
