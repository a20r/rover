# Running
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
