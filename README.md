TermProject468 ReadMe.md
CP468 - Group 15 Term Project 

Overview
This program simulates the navigation of multiple robots in a grid environment.
Each robot must find its way to a designated rendezvous point while avoiding obstacles.
The simulation employs an A* pathfinding algorithm to determine the optimal path for each
robot from its initial position to the rendezvous point.


Files
- Main.java:
  Contains the main classes and the entry point for the simulation.
- Environment.txt:
  Input file defining the grid layout, including the positions of obstacles, the initial
  positions of robots, and the rendezvous point.


Environment Configuration
The environment is defined in a text file (Environment.txt) with the following format:

- First line: Grid dimensions (height and width).
- Second line: Number of robots.
- Subsequent lines: Initial positions of each robot (one per line).
- Line after initial positions: Coordinates of the rendezvous point.
- Remaining lines: Grid layout where 0 denotes an open space and 1 denotes an obstacle.

Example:
8 10
2
2 1
8 2
4 7
1000000001
1100000001
1000000001
1000110001
1000000001
1000000001
1000000001
1000111001


Running the Simulation
1. Compile the Java program:
  javac Main.java

2. Run the program:
  java Main

The program reads the grid configuration from Environment.txt, initializes the simulation,
and displays the moves of each robot step-by-step in the console. The output includes each
robot's path and the time taken to compute the path.


Features
- A Pathfinding*: Utilizes the A* algorithm to find the shortest path from each robot to the
  rendezvous point, taking into account obstacles.
- Obstacle Avoidance: Robots communicate with each other to update their knowledge of the grid,
  marking cells as blocked when obstacles are encountered.
- Simulation Output: Displays detailed information about each step's execution time and the
  overall simulation time.


Dependencies
- Java Development Kit (JDK) to compile and run the program.
- Java's Swing library for optional GUI elements (if implemented in the GUI version).
