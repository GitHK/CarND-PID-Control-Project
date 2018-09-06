# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Installation and running

Please check the original repository 
[https://github.com/udacity/CarND-PID-Control-Project](https://github.com/udacity/CarND-PID-Control-Project)

## PID

PID short for Proportional Integral Derivative controller, mostly used in robotics applications is a piece of software
used to obtain smooth trajectories for robbotics application. It depends on three parameters:

- Kp (Proportional) -> proportional to the current value of the crosstrack_error
- Ki (Integral) -> accounts for past values of the crosstrack_error and integrates them over time to produce 
    the I term
- Kd (Derivative) -> is a best estimate of the future trend of the crosstrack_error, based on its current 
    rate of chang

In our case the output of the controller will be given by the following formula:

    τ = -Kp * crosstrack_error - Ki * integral_error - Kd * derivative_error

## Hyperparameters tuning

For this project I have tried out two approaches:

- twiddle implementation
- manual parameter tuning

The first approach was not successful, and I did not manage to implement an algorithm able to find good parameters. 
It will sort of work if started with good parameters, it will try to tune them out a bit more. Maybe more work should 
have been put in turning on and off the twiddle phase when running the simulation, instead of just running 
only at the start and then have it shut down.

The second approach gave the final result. I would like to point out that the simulator has no system error, thus the 
Ki optimal value is 0.


## Output videos

The following videos provide the final results.

Parameters set manually:

- Kp = 0.1
- Ki = 0.0
- Kd = 3.0

[PID with no twiddle](./output/pid_no_twiddle.mp4)


Final parameters found by "twiddle" implementation:

- Kp = 0.14
- Ki = 0.0
- Kd = 3.0

[PID with twiddle](./output/pid_with_twiddle.mp4)
