# omnipath-control-sim
A modular Python simulation for autonomous vehicle control, implementing Kinematic Bicycle Models and PID-based trajectory tracking.

![PID Recovery Result](docs/images/pid_plot.png)

To achieve stable autonomous tracking, I performed a systematic tuning of the PID gains.The goal was to minimize overshoot while eliminating steady-state error.
Engineering Log:Tuning Trials

Initial Parameters: Kp=0.5, Ki=0.0, Kd=0.1
Result: High overshoot and oscillations.

Second Iteration: Kp=0.3, Ki=0.0, Kd=0.8
Result: Overdamped, smooth but sluggish.

Final Parameters: Kp=0.4, Ki=0.05, Kd=0.8
Result: Stable tracking with minimal overshoot and no steady-state error.

![PID Tuning Result](docs/images/pid_compensated_plot.png)