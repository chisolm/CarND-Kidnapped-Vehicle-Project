# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

The original repository for this project is located at: [CarND-Kidnapped-Vehicle-Project](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

### Attribution

Portions of the code is from my class room exercises from the udacity class on self driving 
cars, lesson 14 "Implementation of a Particle Filter".  The infrastructure code in this project is supplied from project repository listed above.

---

[//]: # (Image References)

[image1]: ./writeup_images/full_track.png "Full view of vehicle track"
[image2]: ./writeup_images/partial_track.png "Show multiple tracks"
[image3]: ./writeup_images/best_particle_weight.png "Best particle weight"
[image4]: ./writeup_images/avg_distance.png "Obs to Landmark associated distance"

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

The run of particle_filter will output a final state in the simulator output that says:

```
Success! Your particle filter passed!
```

##  Sucessful run

The car completes multiple navigations of a single loop:

![alt text][image1]

The car does not share the exact track for each loop as can be seen in the multple lines that it follows in a blown up image of the plotted track.

![alt text][image2]

The best and average particle weights are useful for debug of cases where the 
filter is begining to stray off track.  The weights decay to very very small
values very quickly.

![alt text][image3]

When plotting the average distance from an observation to it's associated 
landmark, there are obvious spikes.  These are from places that the observation
seems to be well outside the expected sensor window and the nearest landmark
is a significant distance away.  This may be due to a mismatch of expected sensor range in the filter used to compute the potential landmarks in range.

![alt text][image4]

#### To use the notebook for limited visualizations

Build and run as listed above:

5. ./particle_filter | tee debug.out 
6. cat debug.out | perl ../src/particle_filter.pl > debug.txt 

Use 'jupyter notebook' or other method to open Debug_particle.ipynb.  It will assume as input the file build/debug.txt generated in step 5 above.

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
    |   Debug_particle.ipynb
    |   particle_filter.pl
```

main.cpp has been modified minimally to supply standard deviations for landmarks in order to pre-compute some values.  Additional output has been
added for visualization.  particle_filter.h also has minimal changes to
pre-compute some values and add a couple debug flags.

## Success Criteria
1. **Accuracy**: My particle filter localizes within the errors checked.

2. **Performance**: My particle filter successfully completes in about 1/2 of the time allowed(appox 50seconds).


