# Scripts

## Scene Generation
To generate a risk map, we have provided an implementation of the diamon-square algorithm
that can be used to create random risk scenes. You are able to specifiy whether the risk
should be modelled by a set of Gaussians or by a random terrain map.

    $ # Generates a risk map
    $ python scripts/create_scene.py [dimension] [name of scene] ["gauss" | "rtm"]
    
## Performance Results
Rover also provide the ability to check graph the performance of a the latest run. The
script uses octave to plot the sensor quality, risk, and total coverage as a function
of the iteration.

    $ # Plots the results from the latest run
    $ octave scripts/generate_sandbox_graphs.m
