Cart and Pendulum Control Simulation
This repository contains a MATLAB implementation of a control simulation for a cart-and-pendulum system. The primary goal is to optimize a control input that drives the system to a specified target point while minimizing the distance traveled.

Overview
The main codes consists of the following sections:

Parameters
M: Mass of the cart (kg)
m: Mass of the pendulum (kg)
l: Length of the pendulum (m)
g: Acceleration due to gravity (m/s^2)
Time Span for Simulation
t_range: Time span for the simulation (start and end times)
Target Point
T: Target point for the tip of the pendulum (specified as [x-coordinate, y-coordinate])
Optimization
Utilizes fmincon to find optimal coefficients for the control input
Initial guess: initial_guess
Optimization algorithm: Sequential Quadratic Programming (sqp)
Simulation
Solves the differential equations describing the system dynamics
Simulates the system with the optimized control input
Extracts and stores the system states over time
Results Illustration
Animation of the cart-and-pendulum system
Plots showing the optimized control input, state variables, and pendulum trajectory
Usage
Run the main.m script.
Observe the animated simulation and plots illustrating the system's behavior and control performance.
Dependencies
MATLAB R2020a or later
License
This code is licensed under the MIT License.

Acknowledgments
The implementation is based on classical control theory and uses optimization techniques for control input optimization.

Feel free to explore, modify, and use this code for educational and research purposes. If you find it helpful, consider giving credit or citing this repository. Contributions and improvements are welcome!
