# LQR-Controlled RPOD Simulation

This repository contains MATLAB code for simulating a Linear Quadratic Regulator (LQR)-controlled system based on the discretized Clohessy-Wiltshire (CW) equations. I am also currently working on code for an optimizer.

## Features
Discretized CW equations for spacecraft dynamics.
LQR controller implemented to compute the full-state feedback gain matrix.
Saturation limits for control input.
Visualization of the state trajectory and control input over time.

## Code Breakdown
The file contains a MATLAB script that simulates an LQR-controlled system based on the discretized Clohessy-Wiltshire (CW) equations.

## File name: 
LQR_CW.m

## Output
The script generates two plots:
    1. Trajectory over time, showing how the system's state evolves.
    2. Control Input over time, showing the control actions applied to maintain stability.

## Requirements
MATLAB (Tested on version 2023b)
