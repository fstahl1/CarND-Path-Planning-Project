The implemented model consists of the three parts *Sensor Fusion*, *Behavior Planning* and *Trajectory Planning*, which are described in detail below:

### 1. Sensor Fusion

In this part, the velocity magnitude as well as d- and s-values are determined for every detected vehicle. To get the current s-values, a prediction was implemented by using the vehicles' velocity, the trajectory timestep and the previous path length.  

Three flags are used to indicate whether 
* the ego vehicle is too close to a car in the same lane in front,
* a left lane change is not feasible, because the lane is blocked,
* a right lane change is not feasible, because this lane is blocked.


### 2. Behavior Planning

The figure below illustrates the implemented finite-state-machine with the following states:
* keep lane:
	The state-machine starts with keeping the current lane and remains in this state until a vehicle is detected in the same lane in front and within the specified distance of 30 meters.
* check for actions
	If a vehicle in front of the car gets too close, the conditions for the following states are checked one after the other.
* Lane change left
	If the left lane is free, which means there is no car in front within 40 meters and no car behind within 10 meters, the ego vehicle changes lane to the left.
* Lane change right
	If the left lane is blocked, but the right lane is free, the car changes lane to the right.
* slow down
	If both left and right lanes are blocked, the car slows down until the distance is large enough again.


![fsm diagram](fsm-diagram.png)


### 3. Trajectory Planning

The trajectory is created created as follows.

To ensure that the new path is tangent to the cars current position, the last two points of the previous path are used as the first points of the new path. If no points are left from the previous path, the current and previous x- and y-positions are used as the first two new points.

To get a smooth trajectory, first only three anchor points are created in 30, 60 and 90 meters distance, which are interpolated with splines later. For simplicity, the three points are defined in Frenet- and then transformed in global x-y-coordinates. Since the splines cannot be calculated for vertical trajectories, the points are transformed to the cars' coordinate system.

The two previous points plus the three new points make five anchor points in total. These five points are then interpolated with splines.

To make sure the spacing of all trajectory points is chosen in a way that the velocity does not exceed 50mph, the method described in the lessons was used. The spacing was calculated using the reference velocity and a defined target distance. After transforming the trajectory points back to global coordinates, they are passed to the simulator.




Note: The coarse structure of this document was created on the basis of [this documentation](https://github.com/Srujankorl/Udacity-SDCEND-HighwayDriving/blob/master/Model%20Documentation.pdf).
