# Highway-Path-Planning

## Introduction

In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed 
limit. The car's localization and sensor fusion data will be provided, there is also a sparse map list of waypoints around the highway. 
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other 
cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes 
at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the 
car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total 
acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

![](https://github.com/Luzhongyue/Highway-Path-Planning/blob/master/view.jpg)

## Data

### Map

The map of the highway is in data/highway_map.txt. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map
coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit 
normal vector pointing outward of the highway loop.The highway's waypoints loop around so the frenet s value, distance along the road, goes
from 0 to 6945.554.

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map
coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet 
coordinates. 

## Usage

### Dependencies

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
    
### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Simulator.

You can download the Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y)
points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in 
the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the 
rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over
10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over
a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not 
very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this 
its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be 
helpful for this transition since they show the last points given to the simulator controller with the processed points already removed.
You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this 
last path.


## Implementation

### 1.Interpolating waypoints of nearby area

Inside data/highway_map.csv there is a list of waypoints that go all the way around the track. The given map is recorded at approximately 30m intervals, a smooth tractory can't not be obtained directly by using getXY function. However, nearby waypoints are interpolated using spline function which can create coarse waypoints with 0.1m interval. By doing this we can get more smooth trajectory in global (x,y) coordinates when mapping trajectory in Frenet frame into trajectory in global coordinate.

### 2.Determine Ego Car Parameters

The edo car parameters include localization Data (car_x,car_y,car_s,car_d) and control data(car_yaw,car_speed). I have defined some vehicle's state including too close(if a car ahead is too close to next car), way too close(too close to need emergency stop), safe_left(safe to change left), safe_right(safe to change right) and lane change(change to target lane).

### 3.Dect nearby vehicle

The position and velocity information of the nearby vehicles can be obtained from the sensor_fusion. According these data, we can difine the state of ego car, ie, it's too close or safe right and so on,so the car should brake or change lane. What's more, these data also transmit to cost function and calculate the best lane.

### 4.Determine the best lane

The path_planner.h has a fuction called Path_planning, which is used to determine the best lane accortiong to current data about ego vehicle and other nerby vehicle.

### 5.Produce new path

The new path starts with a certain number of points from the previous path, which is received from the simulator at each iteration. From there a spline is generated beginning with the last two points of the previous path that have been kept (or the current position, heading, and velocity if no current path exists), and ending with three points 30,60 and 90 meters ahead and in the target lane. This produces a smooth x and y trajectory. To prevent excessive acceleration and jerk, the velocity is only allowed increment or decrement by a small amount, and the corresponding next x and y points are calculated along the x and y splines created earlier.


## Conclusion

The ego vehicle is possible to drive without collisoin under speed limit and jerk even with fairly dense traffic. It can speed up, slow down and change lane accoring to the the path planning fuction.

## Further reading

[Path Planning for Collision Avoidance Maneuver](https://www.researchgate.net/publication/267596342_Path_Planning_for_Collision_Avoidance_Maneuver)
[Optimal Trajectory Planning for Glass-Handing Robot Based on Execution Time Acceleration and Jerk](https://www.hindawi.com/journals/jr/2016/9329131/)
[This discussion on StackExchange can be of interest Which trajectory planning algorithm for minimizing jerk.](https://robotics.stackexchange.com/questions/8555/which-trajectory-planning-algorithm-for-minimizing-jerk)
[Udacity CS373: Programming a Robotic Car Unit 4: Motion](https://www.udacity.com/file?file_key=agpzfnVkYWNpdHl1ckcLEgZDb3Vyc2UiBWNzMzczDAsSCUNvdXJzZVJldiIHZmViMjAxMgwLEgRVbml0GIHQDwwLEgxBdHRhY2hlZEZpbGUYwYUTDA)
[Path Planning and Collision Avoidance](http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf)
[Safe Motion Planning for Autonomous Driving](http://wesscholar.wesleyan.edu/cgi/viewcontent.cgi?article=1856&context=etd_hon_theses)
[Local and Global Path Generation for Autonomous Vehicles Using Splines](http://www.scielo.org.co/pdf/inge/v21n2/v21n2a05.pdf)
[Medium- Path Planning in Highways for an Autonomous Vehicle](Local and Global Path Generation for Autonomous Vehicles Using Splines)
[Real-time motion planning methods for autonomous on-road driving: State-of-the-art and future research directions](http://www.sciencedirect.com/science/article/pii/S0968090X15003447)
[Introduction to Robotics #4: Path-Planning](http://correll.cs.colorado.edu/?p=965)
[The path planning problem in depth](https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume9/mazer98a-html/node2.html)
[A discussion on What is the difference between path planning and motion planning?](https://robotics.stackexchange.com/questions/8302/what-is-the-difference-between-path-planning-and-motion-planning)
[Introduction to robot motion: Robot Motion Planning](http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/18-robot-motion-planning.pdf)
[Introduction to robot motion: Path Planning and Collision Avoidance](http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf)
