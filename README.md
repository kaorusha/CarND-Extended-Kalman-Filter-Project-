# **Extended Kalman Filter**
**Extended Kalman Filter Project**

Utilize a kalman filter to estimate the state of a moving object
* State vector defined as [x, y, x_dot, y_dot] 
* Sensor fusion with lidar and radar data 
* Obtaining RMSE values

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
From the measurement noise covariance matrix R, we can tell that lidar is more accurate(smaller value). we can see that in simulation.
  * Turn off radar (by modify line 111 in `main.cpp`):
```c
if(meas_package.sensor_type_ == MeasurementPackage::LASER){
          fusionEKF.ProcessMeasurement(meas_package);       
}
```
|RMSE | SPEC | Dataset1 | Dataset2
|-----|------|----------|---------
| px  |  .11 |  0.1838  | 0.1656
| py  |  .11 |  0.1542  | 0.1579
| vx  | 0.52 |  0.6051  | 0.6060
| vy  | 0.52 |  0.4858  | 0.4950
  * Turn off lidar (by modify line 111 in `main.cpp`):
```c
if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
          fusionEKF.ProcessMeasurement(meas_package);       
}
```
|RMSE | SPEC | Dataset1 | Dataset2
|-----|------|----------|---------
| px  |  .11 |  0.2319  | 0.2439
| py  |  .11 |  0.3361  | 0.3375
| vx  | 0.52 |  0.5259  | 0.6035
| vy  | 0.52 |  0.6987  | 0.8174
### Follows the Correct Algorithm
#### 1. Follow the general processing flow
  * Create a Kalman Filter instance, `fusionEKF`, in `main()`.(line 36 in main.cpp) 
  * When new sensor data arrived, copy the data into `MeasurementPackage` type and called `FusionEKF::ProcessMeasurement()`(line 111 in main.cpp)

#### 2. Use the first measurements to initialize the state vectors and covariance matrices
  * When the first sensor data arrived, initialize the state vector, `x_`, with first measurement data and create the covariance matrix. (line 74 to 104 in FusionEKF.cpp)
  * Set initial state covariance matrix P, use 1 for a relatively accurate states for px, py, and a high value, like 1000, for relatively unsure states vx ,vy.
  * So the first estimation is equal to the sensor measurement data and get a higher error with the ground truth.

#### 3. First predicts then updates
Upon receiving a measurement after the first, when the second sensor data arrived, calculate the `delta t`, and enter prediction step (line 106 to 134 in FusionEKF.cpp):
  * Calculate new transition matrix F.
  * Calculate new process covariance matrix Q
  * Call `KalmanFilter::Predict()` to predict current state, `x_`, which is object position and velocity.
Then use the sensor measurement data to update the state and covariance matrices (line 135 to 152 in FusionEKF.cpp)
#### 4. Handle radar and lidar measurements
The algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type. 
  * `KalmanFilter::UpdateEKF()` for rader with nonlinear measurement matrix and use the linearized jacobian matrix at the state.(line 50 to 77 in kalman_filter.cpp)
  * `KalmanFilter::Update()` for lidar with linear measurement matrix.(line 25 to 48 in kalman_filter.cpp)

### Code Efficiency
* Store the value and then reuse the value later (line 64 to 66 in tools.cpp) 
* If there is no need to visualize all estimated state and ground truth, we may store the RMSE without memorize all data history.