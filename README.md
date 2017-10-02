# Model Predictive Controller

## Solution Summary
The project uses udacity simulator inputs vehicle position waypoints, x, y, psi, v, delta steering angle and acceleration and predicts steering wheel angle and throttle. Steering wheel angle and throttle are then sent back to simulator for autonomous vehicle driving. Please click below image for output video below.

[![Project Video](http://img.youtube.com/vi/6LirI1wDsIY/0.jpg)](https://youtu.be/6LirI1wDsIY)

Input way points are first converted from global coordinate system to vehicle coordinate system. After waypoints are transformed to vehicle coordinate system a third degree polynomial is being generated. The coefficients of this third degree polynomial is used predict Y coordinates of the vehicle. The coefficients are also used to calculate cost in F_Eval.

The coefficients generated is used to calculate cross track error(cte) and PSI error(epsi).  Cross track error is calculated using polyeval function passing in coefficents and x coordinate as zero. X coordinate of the vehicle is chosen as zero because waypoint coordinates have been transformed to vehicle coordinate system. epsi is calculated as -arctan(f'(x)).

As the coordinate system considered here is vehicle hence the reference coordinates for x, y and psi are zero. State vector is then initialized with x, y, psi, velocity, cte and epsi. Model predictive solver is then called on the above state vector to generate optimized trajectory with throttle and steering wheel angle values.

Model predictive controller class solve function initializes variable array, lower and upper bounds of input variables and lower and upper bounds of constraints. Lower and upper bounds for steering wheel angle is selected as  -0.436332*Lf and 0.436332*Lf respectively. Lower and upper bound for throttle is selected as -1 and 1 respectively.

Initialized vectors are then passed to IOPT solver to optimize input variables. 

To compensate for latency of 100 msec, the inputs to compute transformed waypoints is computed using below kinematic equations. Reference to the approach has been taken from udacity discussions post  https://discussions.udacity.com/t/calibration-for-the-acceleration-and-steering-angle-for-latency-consideration/276413. 

          double latency = 0.1;(100 msec)
          px = px + v*cos(psi)*latency;
          py = py + v*sin(psi)*latency;
          psi = psi - v*(delta/Lf)*latency;
          v = v + accle * latency;


Various values of N and dt has been tried. The most stable value for N has been found to be 10 and dt as 0.1 secs or 100 msec. If N selected is larger car becomes unstable and drives off track. Similarly values other than 0.1 secs results in car being driven off track.

Cost weights of 2000, 2000, 1 5, 200 and 10 has been used for cte, epsi, velocity, delta, accleration, delta_diff and accle_diff respectively. The values used has been referenced from youtube link https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be. Higher weight would make optimizer more sensitive to the respective cost.



## Models

Project solution uses below models as constraints for optimizer. Below modes have been selected based on the udacity tutorials.

![alt Models](https://raw.githubusercontent.com/mrai1983/ModelPredictiveController/master/models.png)


