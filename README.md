# **Extended Kalman Filter**
**Extended Kalman Filter Project**

utilize a kalman filter to estimate the state of a moving object
* state vector defined as [x, y, x_dot, y_dot] 
* sensor fusion with lidar and radar data 
* obtaining RMSE values

## Execution
Build (**Ubuntu** and **Mac**)
from top of the cloned repository:
```bash
$ mkdir build && cd build
$ cmake .. && make
```
Run
```bash
$ ./ExtendedKF
```
Please refer to original project [**instructions**](instruction.md) for detail steps

## Rubric Points
[rubric points](https://review.udacity.com/#!/rubrics/748/view)
### Compiling
Using original [CMakeList.txt](CMakeList.txt) to compile without error with `cmake` and `make`. 
### Accuracy
Result from workspace simulator:
|RMSE | SPEC | Dataset1 | Dataset2
|-----|------|----------|---------
| px  |  .11 |  0.0973  | 0.0726
| py  |  .11 |  0.0855  | 0.0965
| vx  | 0.52 |  0.4513  | 0.4216
| vy  | 0.52 |  0.4399  | 0.4932
#### Analysis: lidar provides more accurate readings
from the measurement noise covariance matrix R, we can tell that lidar is more accurate. we can see that in simulation.
turn off radar:
turn off lidar: 
### Follows the Correct Algorithm
#### 1. Follow the general processing flow
#### 2. Use the first measurements to initialize the state vectors and covariance matrices
#### 3. First predicts then updates
Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.
#### 4. Handle radar and lidar measurements
Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.
### Code Efficiency
store the value and then reuse the value later