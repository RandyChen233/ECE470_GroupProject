*This is the final course project for **ECE470 - Introduction to Robotics** course at the University of Illinois at Urbana-Champaign.*

## How to use
- In a new terminal run **source devel/setup.bash**
- In the same terminal run **roslaunch ur3_driver ur3_gazebo.launch** to open simulation software
- Run **rosrun lab2_pkg.py lab2_spawn.py** to input the blocks in the simulation
- Enter **3** and **n** to input 3 blocks
- Run **rosrun lab2pkg_py lab2_exec.py --simulator True**
- Input **{1,2,3}** for start and end towers 

## Code Walk Through
**Initialize Q matrix**
- Initialize Q which is a 2D array that consists of the robot's orientation at each position(E.g **Q11** is the orientation of the robot when it is at the first tower position with $3$ blocks stacked, **Q21** is the orientation of the robot when it is at the second tower position with $3$ blocks stacked).


**gripper_callback(msg)**
- Takes in the info published from ur3/gripper_input
- gets the state of suction cup based on the input

**position_callback(msg)**
- Callback function thaat takes in the new angle position data published


**gripper(pub_cmd, loop_rate, io_0)**
- 


**move_arm(pub_cmd, loop_rate, dest, vel, accel)**
-

**move_block(pub_cmd, loop_rate, start_loc, start_height, end_loc, end_height)**
-

**main()**
-               
         
