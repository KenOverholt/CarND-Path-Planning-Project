# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are provided.  There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit which means passing slower traffic when possible.  Note that other cars will try to change lanes too. The car should avoid hitting other cars at all costs and should drive inside of the marked road lanes at all times unless switching from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## How I Satisfied the Project's Goals

### Setting the car's goals
For each message that comes in from the simulator, the program loops through all the other cars checking to see if any are within 30 meters ahead of the primary car.  Lines 337-8 look through the cars which are stored in sensor_fusion.

    //find the car in front of me and plan to take action if necessary
    for (int i = 0; i < sensor_fusion.size(); i++) //for each other car on the road

The car's coordinates are measured as Frenet coordinates which are s for the length down the road and d for the offset from the center.  In this case, lane represents the number of the land in which the primary car is riding so the d value from each other car has its compared to see if it is within the width of the primary car's lane.  This comparison is in line 343:

    if ( d>(4*lane) && d<(4*lane+4) ) //left edge of lane is lane*4; right edge of lane is lane*4+4

When another car is found in the lane, it is checked to determine whether it is within 30 meters ahead of the primary car (line 352):

    //check s values greater than mine and s gap
    if ((check_car_s > car_s) && ((check_car_s-car_s) < 30)) //30 meters

If one is found, the program checks whether the car is currently shifting left or right.  It tracks this through a state variable that gets set when a shift starts.  If the car is currently shifting, it checks whether it is still safe to shift.  If it is, it will continue.  If not, it will cancel the shift and stay in its current lane (lines 352-96).

If the car is not currently shifting, it checks whether it is safe to shift one lane left by cycling through all the other cars on the track and checking their Frenet d value to determine whether it is similar to the car's current value.  If so, the other car is next to the car so it can't shift or it will cause a collision.  If left is unavailable, it will check to shift right.  If that is unavailable, it will stay in its current lane.

Next, if the program has determined there is a car too close ahead, it will lower the car's velocity (lines 419-27).

### Planning the car's trajectory
Once the desired lane and velocity are determined, a trajecty is calculated (lines 430-545).  

The high-level approach is to create a list of widely-spaced points waypoints evenly-spaced at 30 meters and then interpolate intermediate points with a spline and fill it in with more points to control the speed.

We either will reference the starting point as where the car is or at the previous paths and point.  If the previous size is almost empty, use the car as starting reference (lines 446-57).  If there are plenty of points, we use the previous path's endpoint as a starting reference (lines 459-75).

In lines 477-489, we use Frenet coordinates to add evenly 30m spaced points ahead of the starting reference.  Shift car reference angle to 0 degrees in lines 490-8.  Next create a spline, fill in the points, and rotate them back to normal (lines 499-544).

### Sample run

#### A full lap with no incidents
![One lap with no incidents](/images/full_lap.png)

#### Switching to the left lane
![Switching left](/images/switching_left.png)

#### Switching to the right lane
![Switching right](/images/switching_right.png)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

