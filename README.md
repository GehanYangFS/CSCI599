# CSCI599 - Class Project
## Authors: Bonan Shen, Gehan Yang

This project aims to build a system towards rendering 3D illuminations using a swarm of drones by leveraging path planning, collision avoidance, failure handling and battery charging methods. The whole project will be conducted in the simulation environment Microsoft AirSim. To narrow the gap between theoretical hypothesis and real-world implementation, the level of availability is something we cannot compromise. In order to satisfy the availability requirements, five modules should be implemented: Architecture, Destination Assignment, Path Planning and Collision Avoidance, Failure Recovery, Battery Charging. The potential contribution of the project can be summarized as:

*  Investigated the use of physics engines to simulate the flight
paths of 3D rendering computed using Artificial Potential Field.
*  Built the dispatcher, charging station and other architectures for drone deployment using Unreal.
*  Implemented algorithms to deploy FLSs to illuminate a single point cloud using MinDist and QuotaBalanced.
*  Introduced USC 3D illumination point cloud.
*  Investigated the use of battery charging to simulate real-world applications with different $\beta$ and $\Omega$ settings, including the value infinite for $\beta$.
*  Introduced FLSs failing and dropping scenes and methods to recover from these failures.
*  Quantified trade-offs to simulate the flight paths computed by ICF versus Artificial Potential Field
*  Open-Sourced a library to formalize any alphabetic 3D illuminations and a library to render any 3D illuminations using AirSim simulation with collision avoidance, failure handling and battery charging.