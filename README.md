# Robotik2627

Repository for participation at the Robocup 2027 in the category Rescue Simulation

## Team

Our team consists of three members: Bastian, Jonas and Luis. This project is being developed as part of our participation in the robotics club at our school in Rahden.

## Competition

The participants program a virtual robot, the e-puck. The robot has to drive through a maze, create a map and recognize victims and dangers with picture detection.\
Website: https://rescuesim.robocup.org/ \
Rules: https://junior.robocup.org/wp-content/uploads/2026/02/RCJRescueSimulation2026-final.pdf

## Features

### Robot
![Picture of robot](./pics/robot.png)\
Sensors: LiDAR, GPS, ultrasonic sensor, cameras, inertial unit, wheel rotation sensors

### Sensor Evaluation

### Mapping

We create the map with a two-dimensional numpy-Array, which is required by the rules. Each element represents a quarter of a 12 cm field.
Mapping is an opportunity to multiply our score, as well as collecting data useful for path finding.

### Driving
