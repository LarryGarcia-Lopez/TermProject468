import java.util.ArrayList;
import java.util.List;

// Class representing a point in a 2D grid
class Point {
    int x, y;

    // Constructor to set the coordinates of the point
    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }

    // Overridden method to provide a string representation of the point
    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}

// Class representing a robot in the simulation
class Robot {
    Point position; // Current position of the robot
    int[][] knownGrid; // The robot's map of the environment including obstacles
    List<Robot> peers; // Other robots in the environment with whom this robot can communicate

    // Constructor to initialize the robot with its position and a copy of the initial grid
    public Robot(int x, int y, int[][] grid) {
        position = new Point(x, y);
        knownGrid = new int[grid.length][grid[0].length];
        for (int i = 0; i < grid.length; i++) {
            System.arraycopy(grid[i], 0, this.knownGrid[i], 0, grid[i].length);
        }
        peers = new ArrayList<>();
    }

    // Method to set the list of peer robots that this robot can communicate with
    public void setPeers(List<Robot> peers) {
        this.peers = peers;
    }

    // Method to communicate detected obstacle data to all peer robots
    public void communicateObstacleData(int x, int y) {
        for (Robot robot : peers) {
            if (robot != this) {
                robot.updateKnownGrid(x, y, 1); // Update the grid of each peer
            }
        }
    }

    // Method to update this robot's known grid with new obstacle information
    public void updateKnownGrid(int x, int y, int value) {
        knownGrid[x][y] = value;
    }

    // Method to move the robot to the next step if possible
    public void move(Point nextStep) {
        if (knownGrid[nextStep.x][nextStep.y] == 0) { // Check if the next step is not an obstacle
            position = nextStep; // Move to next step
        } else {
            communicateObstacleData(nextStep.x, nextStep.y); // Communicate the obstacle if found
        }
    }

    // Method to check if the robot can move to a specified point
    public boolean canMoveTo(Point nextStep) {
        return knownGrid[nextStep.x][nextStep.y] == 0; // Return true if no obstacle at the next step
    }
}

// Class to coordinate multiple robots in a collaborative environment
class CollaborativeRobotCoordinator {
    private int[][] grid; // The shared environment grid
    private List<Robot> robots; // List of robots in the environment
    private Point rendezvous; // The common goal point for all robots

    // Constructor to initialize the coordinator with the environment and robots
    public CollaborativeRobotCoordinator(int[][] grid, List<Robot> robots, Point rendezvous) {
        this.grid = grid;
        this.robots = robots;
        this.rendezvous = rendezvous;
        for (Robot robot : robots) {
            robot.setPeers(robots); // Set each robot's peers to enable communication
        }
    }

    // Method to run the simulation until all robots reach the rendezvous point
    public void runSimulation() {
        boolean allAtRendezvous = false;
        while (!allAtRendezvous) {
            allAtRendezvous = true;
            for (Robot robot : robots) {
                if (!robot.position.equals(rendezvous)) {
                    AStarPathFinder pathFinder = new AStarPathFinder(robot.knownGrid);
                    List<Point> path = pathFinder.findPath(robot.position, rendezvous);
                    if (path != null && path.size() > 1) {
                        Point nextStep = path.get(1);
                        if (robot.canMoveTo(nextStep)) {
                            robot.move(nextStep);
                        }
                    }
                    allAtRendezvous = false; // Continue simulation if any robot hasn't reached the rendezvous
                }
            }
        }
    }
}
