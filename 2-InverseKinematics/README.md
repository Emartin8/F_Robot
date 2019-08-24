# Direct Kinematics

The aim of this TP is to solve the inverse kinematic problem on a 3 joint leg, using simple trigonometry. Then implement the solution.
The implementation will be tested later on a robotic leg.
The key here is to have a good [Kinematic drawing](leg_proj.pdf).


1. Solve the inverse kinematic problem : Knowing P3(x3, y3, z3), L1, L2 and L3, find theta1, theta2, theta3. What is the fundamental difference between the direct and the inverse kinematic problem?

2. Adapt your solution to your robotic leg, i.e. make sure that your solution is valid if you replace motorX.currentPosition by thetaX.

3. Implement your solution using python.

**NOTES:**

Google these if you are in need of a reminder :
- SOH CAH TOA
- Al-Kashi

The answers to 1. and 2. shall be written on a paper version of "leg_proj.pdf". Your work will be collected before the end of the class (1 per student). Clean work expected. A solution will be given afterwards.

Expected format for task 3.:
A file named "inverse_kinematics.py" with a function ```leg_ik(x, y, z, l1=L1, l2=L2, l3=L3, other needed parameters)``` that returns the angles [theta1, theta2, theta3] in ° that need to be applied to the motors in order to reach [x, y , z].
Once the implementation is ready, you'll have to demonstrate its functionnality on the physical robot leg.
