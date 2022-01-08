# Robot Planning and its Application Course - Student Interface
[![Code Documentation](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface.svg)](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface/)

The documentation of the code can be found [HERE](https://codedocs.xyz/lucademenego99/AppliedRoboticsStudentInterface/).

The paper containing the details about the project and the design choices, instead, can be found [HERE](https://drive.google.com/file/d/1TaRlKhSLFmJkzNcFJuVxksPoW7azdRX3/view?usp=sharing).

# Usage
1. Build the simulator using `catkin build`, then `source ~/workspace/simulator/environment.sh` and `source ~/workspace/project/environment.sh`
2. Build the ApplicationRoboticsStudentinterface with `cd build`, `cmake ..` and then `make`, then `source ~/workspace/simulator/environment.sh` and `source ~/workspace/project/environment.sh`
3. Go to `cd $AR_config_dir`, `gedit default_implementation.config` and change the "planning" variable to false (This passage needs to be done only once)
4. In four different terminals, go under the directory `workspace` and run the following commands, one for each terminal:
  - `AR_simulator n:=2` (n is the number of robots to spawn)
  - `AR_pipeline n:=2`
  - `AR_rviz n:=2`
  - `AR_plan`

**[NOTE: ALWAY remember to source the environmet before AR commands, meaning that you need to run `source ~/workspace/simulator/environment.sh` and `source ~/workspace/project/environment.sh`]**
5. Run `AR_plan` and the planned path both for the evader and the pursuer will be visible.
6. Run `AR_run` to move the robots accordingly to the path that was described in the plan phase.


# Structure of our solution
The main elements implemented in order to fulfill the project requirements are:
- **Clipper library**: used for polygon offsetting and join of obstacles.
- **Visibility Graph generation**: generation of the shortest path roadmap. The implemented algorithm follows the plane-sweep principle presented in Section 6.2.2 of the [Motion Planning book](http://lavalle.pl/planning/ch6.pdf) provided to us by the professor. In particular, the chapter 15 of [Computational Geometry - Algorithms and Applications](https://people.inf.elte.hu/fekete/algoritmusok_msc/terinfo_geom/konyvek/Computational%20Geometry%20-%20Algorithms%20and%20Applications,%203rd%20Ed.pdf) has been followed for the details of the visibility graph generation implementation. In order to optimize the algorithm, a special datastructure called OpenEdges has been implemented, following the open-source project [pyvisgraph](https://github.com/TaipanRex/pyvisgraph), written in Python. Final complexity: `O(n^2 logn)`, where `n` is the number of vertices.
- **Dijkstra Shortest Path**: it was implemented using C++ STL sets, which are self-balancing binary search trees. Final complexity: `O(E logV)`, where E is the number of edges and V is the number of vertices of the graph.
- **Multipoint Markov-Dubins Problem**: solved with an Iterative Dynamic Programming approach, explained during class. Final complexity: `O(n k^2)`, where n is the number of points and k is the number of discretised angles.
- **Collision detection**: solved with a basic algorithm to check Arc-Segment intersections, that must be run at each iteration of the Dubins shortest path calculation.
- **Pursuer-Evader game**: a simulation of a pursuer-evader game, in which one robot - the evader - tries to escape to a destination, while another robot - the pursuer - has to intercept it. The details on the developed algorithm can be found on the report linked above.
