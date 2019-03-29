# docker-loco3d
Discussions relative to the docker for the generation of motion data using Loco3D pipeline.

# Documentation
For the complete documentation, please check the following document https://gepgitlab.laas.fr/memmo/deliverables/tree/master/D1.1_Dataset_of_state-control_trajectories  (Only for the members of the MEMMO project).

Below is an overview of the data stored in the current dataset (V0.9). This informations will be subject to change after the release of the dataset V1.0.

# Visualization of the motion

For visualization you will mostly be interested in the joint trajectory q(t). This data is contained in the files \*\_config.csv where each line contains [t q(t)]. Be careful that the first 7 values of q(t) is the position and orientation (Quaternion x,y,z,w) of the root.

TODO : vizualisation script.

## Environment model :

The environment is represented by the following mesh file : https://github.com/humanoid-path-planner/hpp-environments/blob/master/meshes/multicontact/bauzil_stairs.stl

## Robot model : 

We use the model "talos_reduced" from the following package :  https://github.com/stack-of-tasks/talos_data

# Generated Data 

In the generated datasets, all the motions are guarantee to be collision-free, respect the joints limits and the dynamic constraints. In the case of the circle scenario all the motions connect exactly the initial state and the goal state, in the case of the nav bauzil scenario the motion may terminate before the goal state is reached if no valid motion was found to connect the goal.
The dataset contain the following files for each successful runs :

* infos.log give the random configurations sampled (which can be used to re-run this particular problem) and the status of each method.
* contact sequence trajectory.xml contains the sequence of contacts (set of active contacts with positions and normals) with the duration of each contact phase and the centroidal trajectory. This is a serialized class (ContactSequenceHumanoid) that can be loaded with python. See package “multicontact” (https://gepgitlab.laas.fr/loco-3d/multicontact tag V1.0.0 is guarantee to work with the current dataset) for more information.
* TEST NAME config.csv, TEST NAME vel.csv, TEST NAME acc.csv contain the configuration, velocity, acceleration for each join along the trajectory. The first column corresponds to time.
* TEST NAME.hip, TEST NAME.zmp, TEST NAME.pos are files that can be used to test the movement with openHRP (simulation software). 
  * TEST NAME.zmp contains the reference zmp trajectory in the local robot frame.
  * TEST NAME.pos contains the configurations. 
For the three files, each line corresponds to a time step (5 ms by default). The first element of each line corresponds to time, the others to the data (configurations for .pos, zmp position for .zmp).

