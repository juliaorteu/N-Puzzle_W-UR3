#### RA - GIA

<img src="images/logo.png" alt="N-Puzzle logo" width="150">

# N-Puzzle with UR3

## Authors
- Elena Alegret, Esteban Gatein, Asal Mehrabi and Júlia Orteu

## Overview
This project implements tasks for a UR3/UR3e robot manipulator to solve an N-puzzle, inspired by the 15-puzzle, using a visual drawing as the goal. The tasks are designed and simulated with PDDL (Planning Domain Definition Language) and the Kautham planning framework.

<img src="images/BehaviourTree.jpeg" alt="BehaviourTree" width="150">


## Run the PDDL problem

To run the pddl problems, use the following command:

```bash
$ cd puzzle/ff-domains
$  ./ff -o domain.pddl -f problem.pddl > output.txt
```

## Running the Simulation
#### Executing the launch

To run the N-puzzle simulation with the UR3 robot, use the provided launch files. 

```
$ cd catkin_wsTAMP/
$ catkin_make
$ source devel/setup.bash
```
Launch the desired puzzle simulation using one of the provided launch files:
```
$ roslaunch puzzle_PRM.launch
$ roslaunch puzzle_RRT.launch
$ roslaunch puzzle_RRTConnect.launch
```
#### Visualizing the simulation
```
$ kautham-gui
```
Once kautham-gui is open, load the following configuration files:
- OMPL_X_puzzle.xml
- taskfile_tampconfig_puzzle_X.xml
Replace X with the planner you are using (RRT, RRTConnect, or PRM).

## Execute the benchmarking file
To run the benchmark planner, execute the commands
```
$ sudo kautham-console -b bench.xml
$ sudo python3 stats.py result.log -d results.db
```
Interpret results at ![PlannerArena.org](https://plannerarena.org/):
- Solved: Percentatge problems solved.
- Time: Avg. planning time.
- Graph States: Avg. states explored.
- Solution Length: Avg. path length.

## Directory Structure
```
.
├── UR3_Robot.mp4
├── puzzle
│   ├── OMPL_PRM_puzzle.xml
│   ├── OMPL_RRT_puzzle.xml
│   ├── OMPL_RRTconnect_puzzle.xml
│   ├── UR3e
│   │   ├── UR3.py
│   │   ├── conf_extraction.py
│   │   ├── pinza10UR3.py
│   │   ├── pinza40UR3.py
│   │   └── taskfile_tampconfig_puzzle_RRTconnect.xml
│   ├── bench.xml
│   ├── benchmarking
│   │   ├── graph_states.pdf
│   │   ├── result.log
│   │   ├── results.db
│   │   ├── solution_length.pdf
│   │   ├── solved.pdf
│   │   ├── stats.py
│   │   └── time.pdf
│   ├── config
│   ├── controls
│   ├── ff-domains
│   │   ├── domain.pddl
│   │   ├── ff
│   │   ├── manipulation_problem_puzzle
│   │   ├── manipulation_problem_puzzle.ff
│   │   ├── problem.pddl
│   │   └── puzzlemanipulationdomain.pddl
│   ├── obstacles
│   ├── robots
│   ├── tampconfig_puzzle_PRM.xml
│   ├── tampconfig_puzzle_RRT.xml
│   ├── tampconfig_puzzle_RRTconnect.xml
│   ├── taskfile_tampconfig_puzzle_PRM.xml
│   ├── taskfile_tampconfig_puzzle_RRT.xml
│   └── taskfile_tampconfig_puzzle_RRTconnect.xml
├── puzzle_PRM.launch
├── puzzle_RRT.launch
└── puzzle_RRTconnect.launch
```
- **UR3_Robot.mp4**: Video demonstrating the UR3 robot in action solving the puzzle.
- **puzzle**: Directory containing various configuration files and resources for the puzzle tasks.
    - **OMPL_PRM_puzzle.xml**: Configuration for the Probabilistic Roadmap (PRM) planner.
    - **OMPL_RRT_puzzle.xml**: Configuration for the Rapidly-exploring Random Tree (RRT) planner.
    - **OMPL_RRTconnect_puzzle.xml**: Configuration for the RRT connect planner.
    - **UR3e**: Directory related to the UR3e robot configuration.
        - **UR3.py**: Python script defining UR3 robot functionalities.
        - **conf_extraction.py**: Script for extracting configurations.
        - **pinza10UR3.py**: Script for controlling the UR3 gripper (pinza10).
        - **pinza40UR3.py**: Script for controlling the UR3 gripper (pinza40).
    - **bench.xml**: XML file for benchmarking configurations.
    - **benchmarking**: Directory containing benchmarking results and tools.
        - **graph_states.pdf**: Graphical representation of state transitions during benchmarking.
        - **result.log**: Log file generated during benchmarking.
        - **results.db**: Database file storing benchmarking results.
        - **solution_length.pdf**: Graphical representation of solution lengths.
        - **solved.pdf**: Graphical representation of solved states.
        - **stats.py**: Python script for generating statistical analysis from benchmarking results.
        - **time.pdf**: Graphical representation of time taken for benchmarks.
    - **config**: Configuration files for the puzzle simulation.
    - **controls**: Control files for the puzzle simulation.
    - **ff-domains**: Directory containing PDDL domains and problems related to puzzle manipulation.
        - **domain.pddl**: PDDL domain definition for puzzle manipulation.
        - **ff**: Directory containing Fast Forward (FF) planner related files.
        - **manipulation_problem_puzzle**: Specific problem files related to puzzle manipulation.
        - **manipulation_problem_puzzle.ff**: Files associated with FF planner for puzzle manipulation.
        - **problem.pddl**: PDDL problem definition file for the puzzle.
        - **puzzlemanipulationdomain.pddl**: Additional PDDL domain specific to puzzle manipulation.
    - **obstacles**: Configuration files defining obstacles in the puzzle environment.
    - **robots**: Configuration files defining robots used in the puzzle simulation.
    - **tampconfig_puzzle_PRM.xml**: TAMP configuration file specific to the PRM planner.
    - **tampconfig_puzzle_RRT.xml**: TAMP configuration file specific to the RRT planner.
    - **tampconfig_puzzle_RRTconnect.xml**: TAMP configuration file specific to the RRT connect planner.
    - **taskfile_tampconfig_puzzle_PRM.xml**: Task file specific to the PRM planner.
    - **taskfile_tampconfig_puzzle_RRT.xml**: Task file specific to the RRT planner.
    - **taskfile_tampconfig_puzzle_RRTconnect.xml**: Task file specific to the RRT connect planner.
- **puzzle_PRM.launch**: Launch file for running the puzzle with the PRM planner.
- **puzzle_RRT.launch**: Launch file for running the puzzle with the RRT planner.
- **puzzle_RRTconnect.launch**: Launch file for running the puzzle with the RRT connect planner.

---

