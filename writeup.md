# Udacity Self Driving Car Nanodegree 
## Path Planning / Highway Driving Project

[//]: # (Image References)
[image1]: ./fsm.png "Finite State Machine"
[image2]: ./screenshot.png "Success Screenshot"

### Introduction

The aim of this project is to implement a suitable controller to drive a simulator in a highway style environment. It must ensure that the vehicle maintains suitable lane discipline, controls its speed, acceleration and jerk to suitable levels and maintain safe distance to other cars on the road. The simulated vehicle is also required to make progress, attempting suitable lane change and overtaking manoeuvres to pass slower moving vehicles.

The simulator provided for this project is a perfect simulator where the provided path is always achieved. It does not apply any physical limitations on the vehicle hence why speed, acceleration and jerk need to be controlled in the definition of the path provided. 

The path is defined as a series of XY coordinates in the world frame. 

### Lane Control

Before implementing a planning algorithm, the framework was assembled to abstract the controller form the task of maintaining position within a line and at a specific speed. 

As part of the provided code framework, a map of the highway and conversion between Frenet (lane position and distance) and XY coordinates was provided. Combining this with a knowledge of the timesteps of the simulation (20ms), driving along a lane could be easily achieved with a constant lane (D) and incrementing distance (S) in Frenet. Using a suitable spline library (* ref), the path could be smoothed over a suitable length to control levels of jerk with major path points spaced at 30 metres when travelling at 50mph and minor points being derived from the spline and calculated to achieve the appropriate target velocity. 

Furthermore, this lead to the ablility to perform a relatively simple lane change by changing the value of target D. The use of the spline smoothing and suitable steps between major path points means the lane change is achieved in a realistic manner. As a further refinement the major path steps have be linearly adjusted dependent on the current vehicle speed to prevent erratic manouevres are even higher speeds or slow manouevres at lower speeds.

With regard to speed, the path planner maintains a target velocity to try and achieve but in defining the next segment of path it is constrained to only permit a maximum allowable acceleration per time step. This reduces the jerk to acceptable levels. This is a rather crude mechanism and a more thorough speed controller could be implemented in a more realistic scenarion but it performs as required for this task. In addition, this controller does not take into account any lateral acceleration being demanded. If trying to achieve a more performant manoevure, this would need to be considered to ensure total acceleration limits (and tyre capacities) are not exceeded. 

To determine the target velocity, a basic proportional controller has been implemented that tries to maintain a safe gap to the car in front while not exceeding the speed limit (main.cpp lines 227). This can become a little oscillatory if the car in front is not maintaining a steady speed but this is part due to the latency inherent in the system due to the way in which the path controller effectively determines the car speed for the next so-many metres.

### Finite State Machine

Now with a controller model which takes a target lane and velocity, a finite state machine was implemented to definite the lane behaviour of the vehicle. This had to consider issues such as ensuring it did not run into the back of slower vehicles as well as making appropriate decisions on whether it should overtake and whether it was safe to do so.

The state machine implemented is detailed below. For each state the transitions and necessary conditions are detailed along with the fundamental calculations when in each state.


![alt text][image1]


The FSM for this project is relatively simple and has the following states:

`enum State { STAY_IN_LANE, PREP_LEFT_CHANGE, PREP_RIGHT_CHANGE, LEFT_CHANGE, RIGHT_CHANGE };`

The current state is stored in the `CurrentState` variable and a switch statement is used to control program flow (main.cpp lines 129 - 224). In each state, the conditions for moving into a different state are evaluated. If not are successful, the target_lane and target_velocity are set and the path updated.

The states are:

#### STAY_IN_LANE

In this state, the car considers if there is a car in front impeding progess. If it comes across one it will either stay in this state and allow the speed controller to slow down to match the speed of the car infront or it will attempt a lane change manoeuvre to a faster flowing lane. This state is also the entry point of the FSM. To achieve this a function has been provided in the SensorFusionHelper.h file called `IsLaneClear()`, this returns the distance to car directly ahead in the current lane and also the speed at which it is moving.  

#### PREP_LEFT_CHANGE

Having come across a car ahead, the car will prepare to attempt a lane change. Here it will look to ensure the manoeuvre is safe (SensorFusionHelper.h - IsSafeGap function) before moving to the LEFT_CHANGE state. As it is possible for the scenario conditions to change while looking for a safe gap to make the manoeuvre and we do not wish to become stuck in this state, a timer has been implemented such that if no suitable gap is found, the FSM returned ot the STAY_IN_LANE state. Here the decision as to whether to make a lane change or not can be reevaluated.

#### PREP_RIGHT_CHANGE

As above but to the right.

#### LEFT_CHANGE

As entering this state means it has already been determined that it is safe to perform a lane change manoeuvre, this state changes the target lane and increases the target velocity to match that lane.

As that only requires the setting of the path parameters, this state automatically exits straight to the STAY_IN_LANE state.

#### RIGHT_CHANGE

As above but to the left.

### Putting it all together

Combining the FSM, Speed controller and path planner allowed the car to drive acceptably along the highway. There are some areas that could be improved however. For example, the car does not exhibit good behaviour, such as tending to the outside lane if all lanes are empty. Additionally, the controller does not deliberately slow down below the speed of the surrounding cars meaning it can potentially become boxed in and unable to get across to the faster lane in certain circumstances. Ideally here it would be preferable to look further ahead and determine the optimum lane earlier. 

Finally there does appear, visually, to be more scope for tighter manouevres around other vehicles which could be achieved with more tuning of safety factors and improving the path planner to allow it to disgard an existing path. This, however, seems unnecessary as in reality, the distances achieved in the simulation would be comfortable in the real world.

### Final Result

The final controller was successful in being able to control the vehicle in the simulated highway condition. In test runs it ran flawlessly and could have continued indefinitely given the looping nature of the highway provided. 

The vehicle was able to maintain a suitable speed in traffic and made sensible overtake manoeuvres when safe to do so. 

The screenshot below shows the simulation after having achieved XXX miles without incident. 

![alt text][image2]


