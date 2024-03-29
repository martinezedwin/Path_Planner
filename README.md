# Path Planning Project

## Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit, passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Dependencies

This project requires the udacity simulator that can be found [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
Make sure to make the simulator executable.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
---

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Results

The path planner was able to allow the vehicle to navigate through the traffic by adjusting speed and changing lanes to move around traffic when it was necessary and safe for at least 4.32 miles without colliding with any other vehicle. It also followed the 50 mph speed limit, mostly only slowing down if there was a vehicle infront of it without exceeding the max acceleration and jerk.

![For_Miles](./Images/Path_planner_for_miles.png)

A video on how it works can be found here:

[![Final result video](./Images/Path_planner_youtube_video.png)](https://www.youtube.com/watch?v=16HayEl0Xfc&t=2s)


## Reflection

The code model for generating paths can be found under src/main.cpp.

In line 138 - 176: 
It tries to determine where other vehicles are around the ego vehicle. (Infront of us or in the right or left lane)
It also treis to decide if its safe to change lanes in either the right or left direction.

In line 178 - 189:
It reacts to what other vehicles are doing
* If there is vehicle infront, it slows down.
* If there is a vehicle infront and it is safe to lane change left, it will lane change.
* If there is a vehicle infront and a vehicle to the left and it is safe to lane change right, it will lane change.

In line 194 - 299:
It creates a trajectory based on what the vehicle decided to do (slow down, lane change, accelerate, etc.). It does this by initializing a couple of points based on how fast the vehicle should drive (50 mph) and using the spline library to fit a line on those points and following it. It continues to add more points based on the current trajectory to maintain speed required. 




