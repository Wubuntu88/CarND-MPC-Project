## Implementation
This writeup explains the implementation steps I have used to write the Model Predictive Controller

### Model
The model is made up of a state, actuators, and an update equation.
#### State
The state consisted of 6 terms:
* x, y - The x, y position of the car.
* psi - The orientation of the car.
* v - The velocity of the car.
* cte - the cross track error of the car.  This is the difference between the current y position of the vehicle (in vehicle coordinates) and the location where is should be based on its trajectory.
* epsi - the error of the orientation.
#### Actuators
There are two actuator terms.  This means our model can control two terms.
* steering angle
* acceleration ( positive term means accelerate, negative term means break).
#### Update equation
The update equation was taken from the Q&A video (which was taken from the course material).
My update equation is located in MPC.cpp and is shown below.
```C++
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = 
  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = 
  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

### Timestep Length and Elapsed Duration (N & dt)
the N and dt were set to the following:
```C++
size_t N = 10;
double dt = 0.1;
```
These values were taken from the Q&A video and they worked well.
I also tried N = 40, 10, and 5.  
* N = 40 resulted in the car quickly swerving back and forth and off the road.
* N = 20 created a reasonably good model, but the path was predicted farther ahead than necessary.  Also, the car did not handle turns so well.
* N = 5 resulted in the car veering to the right, and kept the car to the right untill it drove into the lake by the bridge.  The green line was short and had the shape of an upside down 'L'.

### Polynomial Fitting and MPC Preprocessing
Before fitting the polynomial, all points were converted from map to vehicle coordinates.
```C++
Eigen::VectorXd ptsx_car(ptsx.size());
Eigen::VectorXd ptsy_car(ptsy.size());
for(unsigned int i = 0; i < ptsx.size(); ++i) {
  double x_shift = ptsx[i] - px;
  double y_shift = ptsy[i] - py;

  ptsx_car[i] = x_shift * cos(0 - psi) - y_shift*sin(0 - psi);
  ptsy_car[i] = x_shift * sin(0 - psi) + y_shift*cos(0 - psi);
}
```
After this conversion, a polynomial fit of order 3 was fit to the points.
```C++
vector<double> next_x_vals;
vector<double> next_y_vals;
double poly_inc = 2.5;
int num_points = 25;
for (int i = 1; i < num_points; ++i) {
  next_x_vals.push_back(poly_inc * i);
  next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
}
```

### Model Predictive Control with Latency
The model sleeps for 100 milliseconds before sending actuator controls to the simulator.  This is to emulate a real world environment.  Each term in the state vector was multiplied by this time delay to account for the latency.
```C++
// Adds a Latency
const double time_delay = 0.1;
// State based on latency info
double delay_x = v * time_delay;
double delay_y = 0;
double delay_psi = v * -old_steering_angle / Lf * time_delay;
double delay_v = v + old_throttle_value * time_delay;
double delay_cte = cte + v * sin(epsi) * time_delay;
double delay_epsi = epsi + v * -old_steering_angle / Lf * time_delay;

Eigen::VectorXd state(6);
state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;

auto vars = mpc.Solve(state, coeffs);
```
I got the idea for this delay modification from this website:
https://medium.com/@cacheop/implementing-a-model-predictive-control-for-a-self-driving-car-7ee6212a04a8