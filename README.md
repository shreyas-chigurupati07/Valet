# Valet - Autonomous parking

## Project Overview

This project aims to simulate a path planning problem for autonomous vehicles, focusing on maneuvering in tight spaces and cluttered environments, particularly while parking. Three vehicles of increasing complexity—a delivery robot, a standard car, and a truck with a trailer—are used to demonstrate the effectiveness of the implemented path planning algorithms.

## Problem Statement

The task involves creating a simulated world where three different vehicles must be parked in compact spaces, taking into account their unique kinematic constraints and avoiding collisions. The vehicles include:

1. A delivery robot with diwheel kinematics.
2. A car with standard Ackerman steering.
3. A truck with a trailer, also with Ackerman steering.

The objective is to develop a kinematic path planner that can efficiently park these vehicles in a specified area while adhering to nonholonomic constraints.

## Algorithm Used

The project employs the Hybrid A* algorithm, a variation of A* that considers kinematic constraints to find neighboring configurations. The algorithm efficiently explores the space by applying a heuristic based on Euclidean distance.

## Implementation Details

- **Environment Setup:** A two-dimensional field simulating a parking lot with obstacles and designated parking spots.
- **Vehicle Models:** Kinematic models of the delivery robot, car, and truck with a trailer are created to simulate their respective steering mechanisms.
- **Path Planning:** The Hybrid A* algorithm is implemented to generate paths for each vehicle, ensuring they reach their parking spots without collisions.

## Results

For each vehicle, the calculated paths for the center of the rear axle are plotted. The movement is depicted through either a short video or superimposed periodic snapshots on an image.
### Car
https://github.com/shreyas-chigurupati07/Valet/assets/84034817/eaded4d7-cb0e-4919-b19c-9ecbb1a04fd0


### Car with Trailer
https://github.com/shreyas-chigurupati07/Valet/assets/84034817/5022c7af-261b-4d56-8c5a-19748301c460


### Differtial Drive
https://github.com/shreyas-chigurupati07/Valet/assets/84034817/9032f5b1-00c5-4592-8319-f5907cdad60d



## Dependencies

- Python 3.x
- Matplotlib (for plotting)
- NumPy (for numerical computations)


## References

- Steven M. LaValle. Planning Algorithms. Cambridge University Press, May 2006.
- Alexey Dosovitskiy et al. CARLA: An open urban driving simulator. Proceedings of the 1st Annual Conference on Robot Learning, pages 1–16, 2017.
