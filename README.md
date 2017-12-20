# CarND-Controls-PID

## Self-Driving Car Engineer Nanodegree Program

### Dependencies

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

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

### Rubric Points

#### Your code should compile.

It does! See 'Basic Build Instructions' above.

Note: `Twiddle.cpp` file was added to `CMakeLists.txt`.

#### The PID procedure follows what was taught in the lessons.

`PID.cpp` contains the PID controller implementation. `PID::TotalError()` is where I calculate total error using P, I, and D gains and errors.
`PID::AccumulatedSquaredError()` accumulates the squared error during the simulation, I use it during PID parameters tuning.
The tuning happens in `Twiddle.cpp` - the instance of this class stores info about which parameter (P, I, or D) is being currently optimized.
`Twiddle::Iterate()` increases/decreases the value of the current parameter.

#### Describe the effect each of the P, I, D components had in your implementation.

The best way to describe the components is to observe the extreme points on this video:

[![Video](https://img.youtube.com/vi/aLXW_RqzfVI/0.jpg)](https://www.youtube.com/watch?v=aLXW_RqzfVI)

* `P` gain is too big - oversteering.
* `P` gain is too small - understeering.
* `I` gain is too big - little effect due absence of persistent errors - slight oversteering.
* `I` gain is too small - no effect in our case since we don't have persistent errors.
* `D` gain is too big - jerky steering.
* `D` gain is too small - cannot compensate `P` oversteering, oscillations last longer.

#### Describe how the final hyperparameters were chosen.

Final parameters (`P` = 0.0837465, `I` = 2.0899e-06, `D` = 1.44656) were chosen after running twiddle for many iterations
(I've lost the number of iterations, it had been running for about an hour). Each iteration runs 2000 simulation steps (~ 20 secs of simulation),
after that the simulation resets to initial state and next iteration of twiddle occurs. I stopped the optimization process after decided current
parameters are good enough.

### The vehicle must successfully drive a lap around the track.

Here's the video:

[![Video](https://img.youtube.com/vi/pkWl3y7p7YE/0.jpg)](https://www.youtube.com/watch?v=pkWl3y7p7YE)
