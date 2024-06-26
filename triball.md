# Triball: a benchmark for a rigid body in contact.

This test simulates a rigid body that is in contact with the ground plane and verifies the contact forces experienced at each contact point.

The model consists of three solid spheres connected by rods in a triangular configuration. This benchmark varies both triball configurations as well as the velocity of the center of mass(com). 
The model doesn't consist of any joints.

Two scenarios have been chosen to validate the simulated solutions and to represent simple and complex scenarios.

In **simple** scenario the triball model is at rest and no external is applied except 
gravitational and contact forces, here the test parameter being the configuration in which balls are arranged (eg: equilateral or isosceles triangle).
The normal/contact force experience at the contact point is a function of triball configuration, which forces can be computed analytically 
and can be used to compare against the simulated/ numerical solution.

In **complex** scenario the triball model has been given a specific initial velocity (both linear and angular). The trajectory of the center of mass(com) and the contact force can calculated
analytically can be used to compare with the simulated/ numerical solution.

## Simple scenario

As the rigid body is at rest, it should not experience any friction force. The forces experienced by it are normal force in the upward direction and gravitational force in the downward direction. 
Which should be equal to each other in magnitude. In an equilibrium state, any rigid body should have a zero net force and torque about its center of mass.
