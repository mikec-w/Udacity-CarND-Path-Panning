# Udacity Self Driving Car Nanodegree 
## Path Planning / Highway Driving Project

### Introduction

The aim of this project is to implement a suitable controller to drive a simulator in a highway style environment. It must ensure that the vehicle maintains suitable lane discipline, controls its speed, acceleration and jerk to suitable levels and maintain safe distance to other cars on the road. The simulated vehicle is also required to make progress, attempting suitable lane change and overtaking manoeuvres to pass slower moving vehicles.

The simulator provided for this project is a perfect simulator where the provided path is always achieved. It does not apply any physical limitations on the vehicle hence why speed, acceleration and jerk need to be controlled in the definition of the path provided. 

The path is definited as a series of XY coordinates in the world frame. 

### Lane Control

Before implementing a planning algorithm, the framework was assembled to abstract the controller form the task of maintaining position within a line and at a specific speed. 

As part of the provided code framework, a map of the highway and conversion between Frenet (lane position and distance) and XY coordinates was provided. Combining this with a knowledge of the timesteps of the simulation (20ms), driving along a lane could be easily achieved with a constant lane (D) and incrementing distance (S) in Frenet. Using a suitable spline library (* ref), the path could be smoothed over a suitable length to control levels of jerk with major path points spaced at 30 metres when travelling at 50mph and minor points being derived from the spline and calculated to achieve the appropriate target velocity. 

Furthermore, this lead to the ablility to perform a relatively simple lane change by changing the value of target D. The use of the spline smoothing and suitable steps between major path points means the lane change is achieved in a realistic manner. As a further refinement the major path steps have be linearly adjusted dependent on the current vehicle speed to prevent erratic manouevres are even higher speeds or slow manouevres at lower speeds.

With regard to speed, the path planner maintains a target velocity to try and achieve but in defining the next segment of path it is constrained to only permit a maximum allowable acceleration per time step. This reduces the jerk to acceptable levels. This is a rather crude mechanism and a more thorough speed controller could be implemented in a more realistic scenarion but it performs as required for this task. 

### Finite State Machine

Now with a controller model which takes a target lane and velocity, a finite state machine was implemented to definite the behaviour of the vehicle. This had to consider issues such as ensuring it did not run into the back of slower vehicles as well as making appropriate decisions on whether it should overtake and whether it was safe to do so.

The state machine implemented is detailed below. For each state the transitions and necessary conditions are detailed along with the fundamental calculations when in each state.


....PICTURE.....


#### States



### Putting it all together


### Final Result

The final controller was successful in being able to control the vehicle in the simulated highway condition. In test runs it ran flawlessly and could have continued indefinitely given the looping nature of the highway provided. 

The vehicle was able to maintain a suitable speed in traffic and made sensible overtake manoeuvres when safe to do so. 



