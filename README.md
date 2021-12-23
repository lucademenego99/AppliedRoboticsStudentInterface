[![Code Documentation](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface.svg)](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface/)

# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. 

# Usage
1. build the simulator using catkin_build, source the environment.sh
2. build the ApplicationRoboticsStudentinterface, source the environment.sh
3. in four terminal, go under the direction xxx/workspace run the following commands sequentially:
AR_simulator n:=3
AR_pipeline n:=3
AR_rviz n:=3
AR_plan
[NOTE: ALWAY remember to source the environmet before AR commands]
4. after running AR_plan, the planed path will shown in rviz, then if want, run AR_run, the robot will go through it!