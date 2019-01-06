PID Controller C++ - Steering Car in Simulation
======
***Project: CarND-Controls-PID - Udacity Self-Driving Car Engineer Nanodegree Program***
**Author: Mario Lüder**

## Overview
This project implements a PID (Proportional-Integral-Derivative) Controller with programming language C++ to steer a  simulated car around a virtual track.

The [simulator](https://github.com/udacity/self-driving-car-sim/releases) provides the cross track error - how far the car is off a given track. This stream of data is used to calculate a steering response to bring the car back on track. Only the steering angle is calculated here, while **driving at maximum speed (50 mph)**.

Also the ["Twiddle" algorithm](https://www.youtube.com/watch?v=2uQ2BSzDvXs&feature=youtu.be), that was introduced by Sebastian Thrun, is implemented here to tune the gain parameters (K<sub>p</sub>, K<sub>i</sub>, K<sub>d</sub> )

## PID Controller
>A **proportional–integral–derivative controller** (**PID controller** or **three-term controller**) is a [control loop](https://en.wikipedia.org/wiki/Control_loop "Control loop")  [feedback mechanism](https://en.wikipedia.org/wiki/Feedback_mechanism "Feedback mechanism") widely used in [industrial control systems](https://en.wikipedia.org/wiki/Industrial_control_system "Industrial control system") and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an _error value_ e(t) as the difference between a desired [setpoint](https://en.wikipedia.org/wiki/Setpoint_(control_system) "Setpoint (control system)") (SP) and a measured [process variable](https://en.wikipedia.org/wiki/Process_variable "Process variable") (PV) and applies a correction based on [proportional](https://en.wikipedia.org/wiki/Proportional_control "Proportional control"), [integral](https://en.wikipedia.org/wiki/Integral "Integral"), and [derivative](https://en.wikipedia.org/wiki/Derivative "Derivative") terms (denoted _P_, _I_, and _D_ respectively), hence the name. ([Wikipedia](https://en.wikipedia.org/wiki/PID_controller))

### Mathematical Form
u(t) = K<sub>p</sub> e(t) + K<sub>i</sub> ∫ e(t') dt' + K<sub>d</sub>  de(t) / dt

wheras:
* K<sub>p</sub> e(t) - is the proportional term
* K<sub>i</sub> ∫ e(t') dt' - is the integral term
* K<sub>d</sub>  de(t) / dt - is the derivative term

#### Proportional Term
The proportional term K<sub>p</sub> e(t)  calculates a steering angle that is proportional to the cross track error. This brings the car back on track. But it causes overshooting, as it steers constantly back and finally over the target position. If K<sub>p</sub> is set too high, the oscillation increases and escalates.

#### Derivative Term
The derivative term helps to avoid overshooting and escalation of oscillation. It is the difference between two errors, the change rate, from previous steps. This reduces or even reverses the steering angle when the car approaches its desired position and the change rate is still high. 

#### Integral Term
The integral term reduces cross track errors coming from an offset. For example the steering angle is set to zero, but the car is not going straight.

## Approach

In order to find the right gain parameters (K<sub>p</sub>, K<sub>i</sub>, K<sub>d</sub> ) a **good guess** must be made. Setting some arbitrary parameters and hoping "twiddle" will find the right ones, does not work.

### Finding a good guess manually
#### Initial guess for K<sub>p</sub>
I have started with setting K<sub>p</sub> = 1.0, K<sub>i</sub> = 0.0, K<sub>d</sub> = 0.0. The maximum speed is set to v<sub>max</sub> = 50 mph and throttle is set to a = 0.3.

As the oscillation escalates quickly, I reduced K<sub>p</sub> by half till it roughly stopped.

| Try | P | I | D | v<sub>max</sub> | a | result |
|--|--|--|--|--|--|--|
| 1 | 1.0 | 0.0 | 0.0 | 50 | 0.3 | Oscillation escalates <br>at the beginning at straight track |
| 2 | 0.5 | 0.0 | 0.0 | 50 | 0.3 | Oscillation escalates <br>at the beginning at straight track at 25 mph|
| 3 | 0.25 | 0.0 | 0.0 | 50 | 0.3 | Oscillation escalates <br>at straight track at 27 mph|
| 4 | 0.12 | 0.0 | 0.0 | 50 | 0.3 | Oscillation escalates <br>at straight track at 30 mph|
| **5** | 0.06 | 0.0 | 0.0 | 50 | 0.3 | Oscillation escalates <br>in curve 32 mph|

The oscillation stopped at the straight part of the track. I would like to get the car faster, so I increased throttle a = 0.5

| Try | P | I | D | v<sub>max</sub> | a | result |
|--|--|--|--|--|--|--|
| 6 | 0.6 | 0.0 | 0.0 | 50 | 0.5 | Oscillation escalates <br>at straight track at 45 mph|
| 7 | 0.3 | 0.0 | 0.0 | 50 | 0.5 |  Oscillation decreases <br>over time, car drives <br>straight but it is not<br>possible to get around<br>the curve with 50 mph|
| **8** | 0.05 | 0.0 | 0.0 | 50 | 0.5 |  Oscillation slightly starts <br>car drives straight but it is not<br>possible to get around<br>the curve with 50 mph|

#### Initial guess for K<sub>d</sub>
I order to reduce this oscillation I guessed that K<sub>d</sub> = 1.0 might help.

| Try | P | I | D | v<sub>max</sub> | a | result |
|--|--|--|--|--|--|--|
| *9* | 0.05 | 0.0 | 1.0 | 50 | 0.5 | **Car gets around the<br>complete track at 50mph**|

As this seems to be a good guess, I kept it.

#### Initial guess for K<sub>i</sub>

I let the car drive a straight part of the track (with previous parameters) and summed up the cross track error. The smaller the sum, the better K<sub>i</sub>.

| Try | K<sub>p</sub> | K<sub>i</sub> | K<sub>d</sub> | v<sub>max</sub> | a | sum(cte) | result |
|--|--|--|--|--|--|--|--|
| 10 | 0.05 | 0.0 | 1.0 | 50 | 0.5 | 139.935 |  |
| 11 | 0.05 | 0.1 | 1.0 | 50 | 0.5 | - | Car makes big turns<br>at the very beginning |
| 12 | 0.05 | 0.01 | 1.0 | 50 | 0.5 | - | Car makes big turns<br>at the very beginning |
| 13 | 0.05 | 0.005 | 1.0 | 50 | 0.5 | - | Car makes big turns<br>at the very beginning |
| **14** | 0.5 | 0.001 | 1.0 | 50 | 0.5 | 21.6695 | Better than attempt 10 |
| 15 | 0.05 | 0.0005 | 1.0 | 50 | 0.5 | 60.6167 | No improvement |

#### Initial parameter set

The final gain parameters are
* K<sub>p</sub> = 0.05
* K<sub>i</sub>  = 0.001
* K<sub>d</sub> = 1.0

It is already possible to get smoothly around the track with these parameters. Lets see if "twiddle" may tune these.

###  Twiddle parameter tuning

The **twiddle algorithm**, also known as “coordinate ascent” is a generic algorithm that tries to find a good choice of parameters, here K<sub>p</sub>, K<sub>i</sub>, K<sub>d</sub>, for an algorithm that returns an error. It does it by successively increasing and decreasing the parameters till the smallest error is found. Sebastian Thrun explains this algorithm in this [video](https://www.youtube.com/watch?v=2uQ2BSzDvXs&feature=youtu.be).

The error is calculated by e = cte<sup>2</sup>. (squared cross track error). *e* is summed up over 1400 sampling steps - a complete round at speed v<sub>max</sub> = 50 mph, throttle a = 0.5.

Full environment set:
* sampling steps before twiddle: 1400 (roughly one track round)
* speed v<sub>max</sub> = 50 mph
* throttle a = 0.5.
* K<sub>p</sub> = 0.05
* K<sub>i</sub>  = 0.001
* K<sub>d</sub> = 1.0
* step<sub>p</sub> = 0.005
* step<sub>i</sub> = 0.0001
* step<sub>d</sub> = 0.1  
* min step<sub>p</sub> = 0.0005
* min step<sub>i</sub> = 0.00001
* min step<sub>d</sub> = 0.01  

#### Final results:
After some hours of driving, twiddle found the following parameters.

| K<sub>p</sub> | K<sub>i</sub> | K<sub>d</sub> | 
|--|--|--|
|**0.166668**|**0.00305867**|**1.82005**|

## Basic Build Instructions

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

### Build
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)