[![Code Documentation](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface.svg)](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface/)

# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. 

# Usage
1. Build the simulator using catkin build, then source ~/workspace/simulator/environment.sh
2. Build the ApplicationRoboticsStudentinterface with cd build, cmake .. and then make, then source ~/workspace/simulator/environment.sh
3. Go to cd $AR_config_dir, gedit default_implementation.config and change the "planning" variable to false
4. In four different terminals, go under the directory xxx/workspace, then run the following commands, one for each terminal:
AR_simulator n:=3
AR_pipeline n:=3
AR_rviz n:=3
AR_plan
[NOTE: ALWAY remember to source the environmet before AR commands]
4. after running AR_plan, the planned path will be shown in rviz, then if want by running AR_run the robot will go through it!
