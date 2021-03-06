# Simulation environment

This directory holds the simulation environment, including the physical model of the WaveRunner and s-function wrappers for
the algorithm implemented (the actual implementation resides under `../code/WR/`).

## Preparation

A few steps need to be taken in order to set up the simulation environment:

- A C++ compiler need to be installed on the system used. The only compiler tested is G++ 6.3.0.
- The [Proj4](https://proj4.org) rel 5.2.0 library need to be installed. There exists newer versions, but these should not be used
due to breaking changes in the API, or the code has to be updated accordingly.

Note that the simulation environment has only been succefully used under Linux (Debian and Ubuntu). Some effort has been made to 
make it compile and run under Windows, but there has been unsolved problems with linking the Proj4 library.

## Usage

In order to use the simulation environment, perform the following steps:

1. Open MATLAB.
2. Right click the `Simulation` directory, click `Add to Path` -> `Selected Folder and Subfolders`.
3. Load `s-functions/build.m` and run both sections in the file to build the s-functions `coords_gen` and `path_alg`. If everything
goes well, `MEX completed successfully.` should be printed in the Command Window. This step has to be repeated whenever
the source files (that is, the C++ code) has been altered.
4. Load `model/wr_data.m` and run it.
5. Right click `model/wr_model_conversion_data.mat`, click `Load`.
6. Load `simulation_environment.slx` in Simulink. Hopefully, it should now be possible to run a simulation.
7. Load `plot_wr_and_coordinates.m` in MATLAB and run it. A plot showing the path generated by the leader craft and the path of
the following WR should appear.

Note that the s-functions and the physical model is packaged in a library (`follow_the_leader_library.slx`). When this file is in
the path, _Follow the leader_ appears in the Library Browser in Simulink.

The s-functions (`Path Following Algorithm / Control system` and `Coordinates generator`) have masks applied, making it easy to
change parameters to the functions by double-clicking the blocks.
