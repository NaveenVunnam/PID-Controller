# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
---
The purpose of this project is to implement a PID controller in C++ to drive the car around the given track in simulator. The simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle. 

### Project Rubric

#### 1.Your code should compile: 
The code compiles without any errors.And no changes were made to CMakeLists.txt.

#### 2. The PID procedure follows what was taught in the lessons
The PID controller is implemented in `/src/PID.cpp`. 

`PID:Init()` method Initializes the PID coefficients and errors.

`PID:UpdateError()` method Updates the PID errors based on CTE(Cross Track Error).

`PID:TotalError()` method Calculates and returns the total error.

#### 3. Describe the effect each of the P, I, D components had in your implementation
P(Proportional) component causes the car to steer proportional to CTE which is the distance of car from center lane. When I use it alone with keeping other components to zero, it causes the car to overshoot.

D(Differential) component tries to counteract the P component's tendency to overshoot the center lane and causes the car to approach the center lane smoothly. 

I(integral) component eliminates the possible bias on CTE which could prevent the PD controller to reach the center lane. When used it alone, it causes the car to go in circles.

#### 4.Describe how the final hyperparameters were chosen.
The final parameters were chosen by trail and error method. First I set all the parameters to zero and Increased the `P component` untill the car starts to oscillate around the center lane steadily. Then I increased the `D component` to remove the oscilations. The `I component` is set to minimize the around big turns. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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




