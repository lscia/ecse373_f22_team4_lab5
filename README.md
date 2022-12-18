# This is the redo of lab 5,6,and 7.


## Luca and Bethany

This redo covers all of lab 5 and lab 6. Lab 7 is still in progress.
This was made in like 3 and a half days.


## LAUNCH INSTRUCTIONS

There is a launch file included called team4_final.launch.


This package should be launched using the following command:

```roslaunch team4_final team4_final.launch```

It has two args: `python` (default to false) and `ecse_launch_file` (defaults to `$(find ecse_373_ariac)/launch/ecse_373_ariac.launch`).

If the launch file does not work, the ecse_373_ariac launch file must be run separately and then this package must be started with:

```rosrun team4_final team4_final_node```


## FUNCTION

The node for this project has a main loop that does the following:
- It keeps track of all orders, shipments, and products.
- It stores all current joint states
- For each order, it goes through all shipments
- For each shipment, it goes through all products
- For each product, it:
  1. Finds the bin that contains the product
  2. Searches the logical camera images to find the pose of the first product
  3. Moves the arm along the linear actuator to the area of the bin
  4. Finds the transform to the part
  5. Gets an IK solution for the first product pose
  6. Checks that solution with forwards kinematics
  7. Moves the arm based on that IK
  8. Resets the arm to the "default" position
  9. Returns the arm to the home along the linear actuator (roughly the center)
- If it is unable to find a suitable IK solution, the arm simply does not attempt to move over the product


## KNOWN ISSUES

This package has several known issues currently:
- A trajectory abort caused by colliding with the environment will sometimes result in unpredictable trajectory behavior
- The arm does not always reach the products, as it will collide with the logical cameras.


## DEPENDENCIES

This package depends on a few others:
- cwru_ariac_2019
- ecse_373_ariac
- osrf_gear
- roscpp
- rviz
- std_msgs
- std_srvs
- sensor_msgs
- ur_kinematics
- actionlib
