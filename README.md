# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
<!-- ### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
``` -->

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Reflection
The path planning development for self-driving car consists of three important components. The components are Prediction, Behavior planning, and Trajectory Generation.

### Prediction
This is the component that estimates the actions a car could take in the future. It uses the current position of the car, it's heading angle, and the velocity, to predict future locations.

The current position, angle, and velocity of the car are readings from the sensore fusion, for example:
Sensor fusion can give the following reading:
```shell
{
    "timestamp" : 34512.21,
    "vehicles" : [
        {
            "id"  : 0,
            "x"   : -10.0,
            "y"   : 8.1,
            "v_x" : 8.0,
            "v_y" : 0.0,
            "sigma_x" : 0.031,
            "sigma_y" : 0.040,
            "sigma_v_x" : 0.12,
            "sigma_v_y" : 0.03,
        },
        {
            "id"  : 1,
            "x"   : 10.0,
            "y"   : 12.1,
            "v_x" : -8.0,
            "v_y" : 0.0,
            "sigma_x" : 0.031,
            "sigma_y" : 0.040,
            "sigma_v_x" : 0.12,
            "sigma_v_y" : 0.03,
        },
    ]
}
```
The output of the prediction module would look like this:

```shell
{
    "timestamp" : 34512.21,
    "vehicles" : [
        {
            "id" : 0,
            "length": 3.4,
            "width" : 1.5,
            "predictions" : [
                {
                    "probability" : 0.781,
                    "trajectory"  : [
                        {
                            "x": -10.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34512.71
                        },
                        {
                            "x": -6.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34513.21
                        },
                        {
                            "x": -2.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34513.71
                        },
                        {
                            "x": 2.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34514.21
                        },
                        {
                            "x": 6.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34514.71
                        },
                        {
                            "x": 10.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34515.21
                        },
                    ]
                },
                {
                    "probability" : 0.219,
                    "trajectory"  : [
                        {
                            "x": -10.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34512.71
                        },
                        {
                            "x": -7.0,
                            "y": 7.5,
                            "yaw": -5.2,
                            "timestamp": 34513.21
                        },
                        {
                            "x": -4.0,
                            "y": 6.1,
                            "yaw": -32.0,
                            "timestamp": 34513.71
                        },
                        {
                            "x": -3.0,
                            "y": 4.1,
                            "yaw": -73.2,
                            "timestamp": 34514.21
                        },
                        {
                            "x": -2.0,
                            "y": 1.2,
                            "yaw": -90.0,
                            "timestamp": 34514.71
                        },
                        {
                            "x": -2.0,
                            "y":-2.8,
                            "yaw": -90.0,
                            "timestamp": 34515.21
                        },
                    ]

                }
            ]
        },
        {
            "id" : 1,
            "length": 3.4,
            "width" : 1.5,
            "predictions" : [
                {
                    "probability" : 1.0,
                    "trajectory" : [
                        {
                            "x": 10.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34512.71
                        },
                        {
                            "x": 6.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34513.21
                        },
                        {
                            "x": 2.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34513.71
                        },
                        {
                            "x": -2.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34514.21
                        },
                        {
                            "x": -6.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34514.71
                        },
                        {
                            "x": -10.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34515.21
                        }
                    ]
                }
            ]
        }
    ]
}

```

The example above shows only vehicles but in reality we will make predictions for all moving objects in the environment, e.g. pedestrians

In this project, we assume we have only 3 lanes and each lane is 4m wide. 

The first task is to know if there is a car in front, at the right, or at the left of our car. We defined three booleans to keep track of that:

```shell
bool car_in_left = false;
bool car_in_right = false;
bool car_ahead = false;
```
The we find the actual lane the car is by using this:

```shell
//check where other cars are
if(d > 0 && d < 4) {
    check_car_lane_pos = 0; // d is between 1 and 3 so car is in first lane
} else if(d > 4 && d < 8) {
        check_car_lane_pos = 1; //d is between 5 and 7 so car is in the second lane
} else if(d > 8 and d < 12) {
    check_car_lane_pos = 2;   //d is between 9 and 11 so car is in the third/last lane
} 
```
So we set our flags to true or false based on the location of the car:

```shell
if(check_car_lane_pos == lane) {
    //the car is on the same lane and in front of the lego car
    //car_ahead |= check_car_s > car_s && (check_car_s - car_s) < 30;	

    if (check_car_s > car_s && (check_car_s - car_s) < 30)
    {
    car_ahead = true;
    }							

} else if((check_car_lane_pos - lane) == -1) {
    //the car is on the left lane of the lego car
    //car_left |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

    if ((car_s+30) > check_car_s  && (car_s-30) < check_car_s)
    {
        car_in_left = true;
    }

} else if((check_car_lane_pos - lane) == 1) {
    //A vehicle is on the right lane and check that is in 30 meter range
    //car_right |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

    if ((car_s+30) > check_car_s  && (car_s-30) < check_car_s)
    {
    car_in_right = true;
    }

}

```

### Behavior Planning
This component determines the action our ego car should in time based on the location of other cars.
Our ego can slow down if the car at its front is moving slow, can change left to left if the left lane is free, can change lane to right if right lane is free, or acclerate if there is enough space at the front and the max. speed is not reached.
The logic for that is :
```shell

f(car_ahead) {
    if(!car_in_left && lane > 0) {
        lane--; //if there is no car in left lane amd the current lane is not left lane, change to left lane
    } else if(!car_in_right && lane !=2) {
        lane++; //if there is no car in right lane amd the current lane is not right lane, change to left lane
    } else if(!car_in_left && lane !=2) {
        lane++; //if there is no car in leftmost lane amd the current lane is not leftmost lane, change to right lane
    }else {
        ref_vel -= change_speed; // decrease speed if changing lane is not posibble
    }
    } 
    else if(ref_vel < max_speed){
        ref_vel += change_speed; //accelerate if no car in front
    }

```

### Trajectory Generation
This component dtermines the best trajectory to follow from a present location to a desired location.
We first find the size of the previous points we have:
```shell
int prev_size = previous_path_x.size();
```

We then keep the reference x, y, and yaw points:
```shell
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);
```

If the previous points is almost empty, we use the current car's position to determine the previous points and add them to the list:

```shell
if (prev_size < 2) 
    {
    //Use two points that make the path tanget to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

    }
```

If we have enough previous points, we add two previous point to the list:

```shell

//Redefine reference state as previous path end point
ref_x = previous_path_x[prev_size-1];
ref_y = previous_path_y[prev_size-1];

double ref_x_prev = previous_path_x[prev_size-2];
double ref_y_prev = previous_path_y[prev_size-2];
ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

//Use the two points that make the path tangent to the previous path'snd point
ptsx.push_back(ref_x_prev);
ptsx.push_back(ref_x);

ptsy.push_back(ref_y_prev);
ptsy.push_back(ref_y);
```
Now we need to add 3 future points to ptsx, psy vecotrs:

```shell
//In Frenet add evenly 30m spaced points ahead of the starting reference
vector <double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector <double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector <double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

For trajectory generation, we are using spline:
```shell
//create a spine
tk::spline s;

//set (x,y) points to the spine
s.set_points(ptsx,ptsy);
```

Then we add all previous points to next_x_vals and next_y_vals:
```shell

//Define the actual (x,y) points we will use for the planner
vector<double> next_x_vals;
vector<double> next_y_vals;

//Start with all the previous path points from the last time
for (int i = 0; i < previous_path_x.size(); i++)
{
next_x_vals.push_back(previous_path_x[i]);
next_y_vals.push_back(previous_path_y[i]);
}
```

For the ego car to travel at the desired speed, we need to find the all spline points till the horizon(say 30m) value:
```shell

//Calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
```

Finaly can calculate the spline points from start to horizon y points:
```shell
//Fill up the rest of our path lanner after filling it with previous points, here we will always output 50 points
for (int i = 0; i <= 50-previous_path_x.size(); i++)
{
double N = (target_dist/(.02*ref_vel/2.24));
double x_point = x_add_on+(target_x)/N;
double y_point = s(x_point);

x_add_on = x_point;

double x_ref = x_point;
double y_ref = y_point;

//rotate back to normal after rotating it eailer
x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

x_point+= ref_x;
y_point+=ref_y;


next_x_vals.push_back(x_point);
next_y_vals.push_back(y_point);


} 

```

<!-- #### The map of the highway is in data/highway_map.txt
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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
 -->
