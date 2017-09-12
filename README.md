# Particle Filter Project
Self-Driving Car Engineer Nanodegree Program

This repository utilizes a particle filter to localize a moving vehicle in a simulation. 

## Inputs
1. Velocity and yaw rate as control data for motion prediction
2. Sets of x, and y relative distance to landmarks obtained from vehicle sensors

## Outputs
1. Most likely position x, and y of vehicle on global map coordinates

## Simulator environment
Udacity's Self-Driving Car's Term 2 Simulator


[//]: # (Image References)

[image1]: SimPF_no_asso.gif "Particle filter without association"
[image2]: SimPF.gif "Particle filter with association"

## Demonstration on Udacity's Term 2 Simulator

### Particle filter without data association
![alt text][image1]
### Particle filter with data association
![alt text][image2]


## Mechanism

General steps used in particle filter
1. Generate initial sets of particle from initial measurement with added random gaussian noise
2. For each time step - predict particle position from control data
3. For each time step - add motion noise
4. For each time step - convert observations from vehicle coordinate to map coordinate
5. For each time step - for each particle, determine likelihood of observations occuring given particle state
6. For each time step - rebuild particles collection by resampling from likelihood determined previously
7. For each time step - report particle with highest weight
8. Repeat steps 2 to 8


## Compile and running the simulator

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). Once the necessary prerequisites are completed, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter


## Detailed inputs/outputs of simulator and ./particle_filter

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"] 

["sense_y"] 

["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"] 

["sense_observations_y"] 


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label 

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions




