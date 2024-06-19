#### RA - GIA

<img src="images/logo.png" alt="N-Puzzle logo" width="150">

# N-Puzzle with UR3

## Authors
- Elena Alegret
- Esteban Gatein
- Asal Mehrabi
- Júlia Orteu

## Project Description
This project implements tasks for a UR3/UR3e robot manipulator to solve an N-puzzle, inspired by the 15-puzzle, using a visual drawing as the goal. The tasks are designed and simulated with PDDL (Planning Domain Definition Language) and the Kautham planning framework.

## Run the PDDL problem

To run the pddl problems, use the following command:

```bash
$ cd puzzle/ff-domains
$  ./ff -o domain.pddl -f problem.pddl > output.txt
```

## Running the Simulation

#### Executing the launch

To run the N-puzzle simulation with the UR3 robot, use the provided launch files. For example:

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

## Directory Structure

```
.
├── README.md
├── UR3_Robot.mp4
├── puzzle
│   ├── OMPL_PRM_puzzle.xml
│   ├── OMPL_RRTconnect_puzzle.xml
│   ├── UR3e
│   ├── config
│   ├── controls
│   ├── ff-domains
│   ├── obstacles
│   ├── robots
│   ├── tampconfig_puzzle_PRM.xml
│   ├── tampconfig_puzzle_RRTconnect.xml
│   ├── taskfile_tampconfig_puzzle_PRM.xml
│   └── taskfile_tampconfig_puzzle_RRTconnect.xml
├── puzzle_PRM.launch
└── puzzle_RRTconnect.launch
```

- **UR3_Robot.mp4**: Video demonstrating the UR3 robot in action solving the puzzle.
- **puzzle**: Directory containing various configuration files and resources for the puzzle tasks.
    - **OMPL_PRM_puzzle.xml**: Configuration for the Probabilistic Roadmap (PRM) planner.
    - **OMPL_RRTconnect_puzzle.xml**: Configuration for the Rapidly-exploring Random Tree (RRT) connect planner.
    - **UR3e**: Directory related to the UR3e robot configuration.
    - **config**: Configuration files for the simulation.
    - **controls**: Control files for the robot.
    - **ff-domains**: Directory containing PDDL domains and problems.
    - **obstacles**: Configuration files for obstacles in the environment.
    - **robots**: Configuration files for robots used in the simulation.
    - **tampconfig_puzzle_PRM.xml**: TAMP configuration file for the PRM planner.
    - **tampconfig_puzzle_RRTconnect.xml**: TAMP configuration file for the RRT connect planner.
    - **taskfile_tampconfig_puzzle_PRM.xml**: Task file for the PRM planner.
    - **taskfile_tampconfig_puzzle_RRTconnect.xml**: Task file for the RRT connect planner.
- **puzzle_PRM.launch**: Launch file for running the puzzle with the PRM planner.
- **puzzle_RRTconnect.launch**: Launch file for running the puzzle with the RRT connect planner.

---

