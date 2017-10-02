# Model Predictive Controller

## Solution Summary
The project uses udacity simulator inputs vehicle position waypoints, x, y, psi, v, delta steering angle and acceleration and predicts steering wheel angle and throttle. Steering wheel angle and throttle are then sent back to simulator for autonomous vehicle driving. Please click below image for output video below.

[![Project Video](http://img.youtube.com/vi/6LirI1wDsIY/0.jpg)](https://youtu.be/6LirI1wDsIY)

Input way points are first converted from global coordinate system to vehicle coordinate system. After waypoints are transformed to vehicle coordinate system a third degree polynomial is being generated. The coefficients of this third degree polynomial is used predict Y coordinates of the vehicle. The coefficients are also used to calculate cost in F_Eval.

The coefficients generated is used to calculate cross track error(cte) and PSI error(epsi).  Cross track error is calculated using polyeval function passing in coefficents and x coordinate as zero. X coordinate of the vehicle is chosen as zero because waypoint coordinates have been transformed to vehicle coordinate system. epsi is calculated as -arctan(f'(x)).

As the coordinate system considered here is vehicle hence the reference coordinates for x, y and psi are zero. State vector is then initialized with x, y, psi, velocity, cte and epsi. Model predictive solver is then called on the above state vector to generate optimized trajectory with throttle and steering wheel angle values.

Model predictive controller class solve function initializes variable array, lower and upper bounds of input variables and lower and upper bounds of constraints. Lower and upper bounds for steering wheel angle is selected as  -0.436332*Lf and 0.436332*Lf respectively. Lower and upper bound for throttle is selected as -1 and 1 respectively.

Initialized vectors are then passed to IOPT solver to optimize input variables. 

To compensate for latency of 100 msec, the inputs to compute transformed waypoints is computed using below kinematic equations 

          double latency = 0.1;(100 msec)
          px = px + v*cos(psi)*latency;
          py = py + v*sin(psi)*latency;
          psi = psi - v*(delta/Lf)*latency;
          v = v + accle * latency;


## Models

Project solution uses below models as constraints for optimizer. Below modes have been selected based on the udacity tutorials.

![alt Models](https://raw.githubusercontent.com/mrai1983/ModelPredictiveController/master/models.png)


