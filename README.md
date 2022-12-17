#This is the redo of lab 5,6,and 7.

##Luca and Bethany


There is a launch file included called team4_final.launch.

This package should be launched using the following command:
roslaunch team4_final team4_final.launch
It has two args: python (default to false) and ecse_launch_file (defaults to $(find ecse_373_ariac)/launch/ecse_373_ariac.launch).
Currently the launch file has issues finding the ecse_373_ariac package despite it being able to launch separately.


The node for this project has a few functionalities:
It keeps track of all orders, shipments, and products.
It stores all current joint states
It finds the bins in which products are located and searches through the associated cameras to find the pose of each product.
It finds the pose of each product with respect to the arm base and adds an offset for the end effector.