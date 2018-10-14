# Model Predictive Control

Part of the Self-Driving Car nanodegree. Starter code and data from [Udacity](https://github.com/udacity/CarND-MPC-Project/).

The simulator is available [here](https://github.com/udacity/self-driving-car-sim)

## Build
- `#>` `mkdir build && cd build && cmake .. -DGIT_SUBMODULE=ON && make`
- `#>` `./MPC`

## The Model

The model takes values from the simulator and consists of the [State](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/MPC.hpp#L25-L32):
 - x, y: the current position
 - &psi; (psi): the current orientation
 - v: the current velocity
 - cte: the cross-track error
 - e&psi; (epsi): the orientation angle error
 
and the [Actuators](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/MPC.hpp#L34-L37):
- a: the throttle
- delta: the steering angle
 
To update the model, we use the following [equations](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/MPC.cpp#L153-L168):
- x<sub>t</sub> = x<sub>t-1</sub> + v<sub>t-1</sub> * cos(&psi;<sub>t-1</sub>) * dt
- y<sub>t</sub> = y<sub>t-1</sub> + v<sub>t-1</sub> * sin(&psi;<sub>t-1</sub>) * dt
- &psi;<sub>t</sub> = &psi;<sub>t-1</sub> + v<sub>t-1</sub> / L<sub>f</sub> * &delta;<sub>t-1</sub> * dt
- v<sub>t</sub> = v<sub>t-1</sub> + a<sub>t-1</sub> * dt
- cte<sub>t</sub> = f(x<sub>t-1</sub>) - y<sub>t-1</sub> + v<sub>t-1</sub> * sin(e&psi;<sub>t-1</sub>) * dt
- e&psi;<sub>t</sub> = &psi;<sub>t</sub> - &psi;des<sub>t-1</sub> + v<sub>t-1</sub> * &delta;<sub>t-1</sub> / L<sub>f</sub> * dt
  
Where dt is the time difference in seconds.

## Pre-processing

To prepare the model for the MPC solver, the [values provided](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/main.cpp#L55-L60) 
from the simulator are processed:
1. The x and y points from the reference path are transformed (using the car's current position) from the simulator's coordinate space to the car's coordinate space:
    ```
    xPoints(i) = ptsx[i] - px * cos(-psi) - ptsy[i] - py * sin(-psi);
    yPoints(i) = ptsx[i] - px * sin(-psi) + ptsy[i] - py * cos(-psi);
    ``` 
1. These points are then used to fit a 3rd order polynomial
1. The cross-track error (cte) is than calculated as the difference between the reference path and the car's position at `x=0` 
(y coordinate from the car's perspective is 0, so the polynomial at `x=0` is the cte)
1. The orientation error (epsi) is calculated as the difference between the actual orientation and the reference path's 
orientation at `x=0` (`-atan(coeffs[1])`)

This forms the input state for the MPC (ignoring the latency). x, y and psi are set to 0 as the car is the reference point 
for the MPC's calculations.

## Latency
To deal with the (simulated) latency, we take the simulator's values and predict them at the point where the car is likely 
to be given the current state. To do this, the same update [equations](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/MPC.cpp#L153-L168) 
are used as are in the [MPC](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/MPC.cpp#L121-L126) (see above).

This can be found in [main.cpp](https://github.com/ivovandongen/CarND-term2-MPC-Project/blob/master/src/main.cpp#L83-L87):
```
const int latency_ms = 100;
const double latency_s = latency_ms / 1000.0;
auto state = mpc.predict({0, 0, 0, v, cte, epsi}, {-(steering_angle), throttle}, latency_s);
```


## Timestep Length and Elapsed Duration (N & dt)

Initially, I started out with the timestep length and duration parameters from the course (N=25 and dt=0.05). 
This turned out to be too heavy and resulted in very erratic behaviour. Before taking latency into account, 
I settled on N=10 and dt=0.2, a 2 second horizon. When taking into account latency though, this was still a little
erratic and I lowered the duration to 0.1, settling on a 1 second horizon (N=10 and dt=0.1).