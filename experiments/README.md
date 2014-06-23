## How to read experimental results

### File Structure
    [date]-[algorithm]/
        data/
        imgs/
        positions/
        config.json

### `data/`
Stores the experimental results for sensor coverage, sensor quality, and average risk.
All files in `data/` are named like `out_[number of risk points]_[number of quads].txt`.
Each file is structured by columns where the first column is the sensor coverage, the
second column is the sensor quality, and the third column is the average risk.

### `imgs/`
Stores heatmap plots of the different risk scenarios that were randomly generated. All the
files in data are named like `heatmap_[number of risk points].png`.

### `positions/`
Stores all of the position data of the quads during a given experiment. The files are named
like `pos_[number of risk points]_[number of quads].csv`. The columns in order are the
iteration number, the quad index, the x position, the y position, the z position, and the sensor
radius.

### `config.json`
This file shows the setup parameters used for the experiments.

## How to generate the graphs

There is a Octave script, `scripts/generate_graphs.m`, that will automatically generate the
graphs assuming that the script is being run from the directory that the experimental
results are stored. Below is some code that shows how the graphs can be generated.

    cd experiments/[date of the experiment]-[algorithm name]
    octave ../../scripts/generate_graphs.m

## Experiment instance descriptions
- Mon June 2 6:59:11 Rover Gaussian
    - Experiments without changing orientation
    - Down facing camera

- Mon June 2 6:59:21 Lawnmower
    - Lawnmower experiments without changing orientation

- Tue June 3 11:17:54 Rover Gaussian
    - Experiments that plan for changes in orientation
    - Camera is at an angle

- Thu June 5 10:33:31 Rover Gaussian
    - Experiments that plan for orientation and camera angle
    - Camera is set at a variable angle.

- Thu June 5 15:23:19 Rover Gaussian
    - Experiments that were run on a larger area (2000x2000) to show
    that the algorithm scales for different sized environments
    - Camera is set at a variable angle and therefore the planner plans
    for *x, y, z*, the camera angle, and the orientation

- Thu June 5 15:27:37 Rover Gaussian
    - Experiments that were run on a smaller area (600x600) to show
    that the algorithm scales down
    - Camera is set at a variable angle and therefore the planner plans
    for *x, y, z*, the camera angle, and the orientation

- Mon June 23 11:11:33 Rover Gaussian
    - Experiments with a fixed camera angle of 15 degrees
    - Also gathering time difference data in the time grid

- Mon June 23 11:11:51 Rover Gaussian
    - Experiments with a fixed camera angle of 10 degrees
    - Also gathering time difference data in the time grid

- Mon June 23 11:12:01 Rover Gaussian
    - Experiments with a fixed camera angle of 20 degrees
    - Also gathering the time difference data in the time grid

- Mon June 23 11:12:10 Rover Gaussian
    - Experiments with a fixed camera angle of 0 degrees
    - Also gathering the time difference data in the time grid

