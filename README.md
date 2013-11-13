dumbo_hardware_interface
========================

ROS interface and wrappers for hardware components of Dumbo.

For bringin up the robot manipulator + sensors please look at the dumbo_bringup package.


dumbo_powercube_chain
-----------------------------------

Does the low level control of Dumbo's Schunk manipulators and provides high level ROS interface.


Based on the `schunk_powercube_chain` package from Fraunhofer IPA (http://wiki.ros.org/schunk_powercube_chain)



dumbo_force_torque_sensor
-----------------------------------

Publishes the wrist-mounted force-torque sensor readings on Dumbo as ROS messages.