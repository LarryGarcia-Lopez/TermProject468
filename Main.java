import java.io.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

// Class representing a coordinate point on a grid
class Point {
    int x, y;    // x and y coordinates of the point

    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }
    
    // Returns a string representation of the point
    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    
    // Checks if two points are equal based on their coordinates
    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null || getClass() != obj.getClass())
            return false;
        Point point = (Point) obj;
        return x == point.x && y == point.y;
    }
}

// Class representing a Robot with its own position, known grid, and peers
class Robot {
    Point position; // Current position of the robot
    int[][] knownGrid; // The grid known by the robot
    List<Robot> peers; // Other robots in the simulation
    Color color; // Color of the robot, used for GUI representation
    List<Point> currentPath; // Current path the robot is following

    public Robot(int x, int y, int[][] grid, Color color) {
        position = new Point(x, y);
        knownGrid = new int[grid.length][grid[0].length];
        for (int i = 0; i < grid.length; i++) {
            System.arraycopy(grid[i], 0, this.knownGrid[i], 0, grid[i].length);
        }
        peers = new ArrayList<>();
        this.color = color;
        this.currentPath = new ArrayList<>();
    }
    
    // Sets the list of peer robots
    public void setPeers(List<Robot> peers) {
        this.peers = peers;
    }

    // Communicates the presence of an obstacle to other robots
    public void communicateObstacleData(int x, int y) {
        for (Robot robot : peers) {
            if (robot != this) {
                robot.updateKnownGrid(x, y, 1);
            }
        }
    }

    // Updates the robot's known grid with new data
    public void updateKnownGrid(int x, int y, int value) {
        knownGrid[x][y] = value;
    }

    // Moves the robot to a new point if valid
    public void move(Point nextStep) {
        if (isValidMove(nextStep)) {
            position = nextStep;
        } else {
            for (Robot r : peers) {
                r.updateKnownGrid(nextStep.x, nextStep.y, 1);
            }
        }
    }
    
    // Checks if the robot can move to a specified point
    public boolean canMoveTo(Point nextStep) {
        return isValidMove(nextStep);
    }

    // Helper method to determine if a move is valid based on the known grid
    private boolean isValidMove(Point p) {
        if (p.x < 0 || p.x >= knownGrid.length || p.y < 0 || p.y >= knownGrid[0].length) {
            return false;
        }
        return knownGrid[p.x][p.y] == 0;
    }
}

// Class for finding paths using the A* pathfinding algorithm
class AStarPathFinder {
    private int[][] grid;
    private int rows, cols;
    private long totalTime;

    public AStarPathFinder(int[][] grid) {
        this.grid = grid;
        this.rows = grid.length;
        this.cols = grid[0].length;
        this.totalTime = 0;
    }

    private class Node implements Comparable<Node> {
        Point point;
        Node parent;
        double g; // Cost from the start node
        double h; // Heuristic cost to the goal
        double f; // Total cost (g + h)

        public Node(Point point, Node parent, double g, double h) {
            this.point = point;
            this.parent = parent;
            this.g = g;
            this.h = h;
            this.f = g + h;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.f, other.f);
        }
    }
    
    // Heuristic function using the Manhattan distance
    public double manhattanHeuristic(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    // Method to find the optimal path from start to goal using A*
    public List<Point> findPath(Point start, Point goal) {
        long startTime = System.nanoTime();
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        boolean[][] closedSet = new boolean[rows][cols];

        Node startNode = new Node(start, null, 0, manhattanHeuristic(start, goal));
        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (current.point.equals(goal)) {
                long endTime = System.nanoTime();
                List<Point> path = reconstructPath(current);
                totalTime += (endTime - startTime);
                System.out.println("Execution Time (ms): " + (totalTime) / 1e6);
                System.out.println("Path Length: " + path.size() + "\n");
                System.out.println("Optimal Path: " + path);
                return path;
            }

            closedSet[current.point.x][current.point.y] = true;

            int[][] directions = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
            for (int[] dir : directions) {
                int newX = current.point.x + dir[0];
                int newY = current.point.y + dir[1];

                if (newX < 0 || newX >= rows || newY < 0 || newY >= cols)
                    continue;
                if (grid[newX][newY] == 1)
                    continue;
                if (closedSet[newX][newY])
                    continue;

                Point neighborPoint = new Point(newX, newY);
                double tentativeG = current.g + 1;
                double h = manhattanHeuristic(neighborPoint, goal);
                Node neighbor = new Node(neighborPoint, current, tentativeG, h);

                openSet.add(neighbor);
            }
        }
        System.out.println("No path found.");
        return new ArrayList<>();
    }

    // Reconstructs the path from the goal to the start using parent links
    private List<Point> reconstructPath(Node node) {
        List<Point> path = new ArrayList<>();
        while (node != null) {
            path.add(0, node.point);
            node = node.parent;
        }
        return path;
    }
}

// Class coordinating multiple robots in a collaborative simulation
class CollaborativeRobotCoordinator {
    private int[][] grid;
    private List<Robot> robots;
    private Point rendezvous;
    private RobotSimulationGUI gui;

    public CollaborativeRobotCoordinator(int[][] grid, List<Robot> robots, Point rendezvous, RobotSimulationGUI gui) {
        this.grid = grid;
        this.robots = robots;
        this.rendezvous = rendezvous;
        this.gui = gui;
        for (Robot robot : robots) {
            robot.setPeers(robots);
        }
    }

    // Steps the simulation, returning true if not all robots are at the rendezvous
    public boolean stepSimulation() {
        boolean allAtRendezvous = true;
        for (Robot robot : robots) {
            if (!robot.position.equals(rendezvous)) {
                AStarPathFinder pathFinder = new AStarPathFinder(robot.knownGrid);
                List<Point> path = pathFinder.findPath(robot.position, rendezvous);
                robot.currentPath = path;

                if (path != null && path.size() > 1) {
                    Point nextStep = path.get(1);
                    if (robot.canMoveTo(nextStep) &&
                        (grid[nextStep.x][nextStep.y] == 0 || nextStep.equals(rendezvous))) {
                        robot.move(nextStep);
                    } else {
                        for (Robot r : robots) {
                            r.updateKnownGrid(nextStep.x, nextStep.y, 1);
                        }
                    }
                }
                allAtRendezvous = false;
            }
        }
        return !allAtRendezvous;
    }

    // Runs the entire simulation until all robots reach the rendezvous
    public void runSimulation() {
        long totalStartTime = System.nanoTime(); // Start timing the entire simulation

        boolean allAtRendezvous = false;
        while (!allAtRendezvous) {
            long stepStartTime = System.nanoTime(); // Start timing each step

            allAtRendezvous = true;
            for (Robot robot : robots) {
                if (!robot.position.equals(rendezvous)) {
                    AStarPathFinder pathFinder = new AStarPathFinder(robot.knownGrid);
                    List<Point> path = pathFinder.findPath(robot.position, rendezvous);
                    robot.currentPath = path;

                    if (path != null && path.size() > 1) {
                        Point nextStep = path.get(1);
                        if (robot.canMoveTo(nextStep) &&
                            (grid[nextStep.x][nextStep.y] == 0 || nextStep.equals(rendezvous))) {
                            robot.move(nextStep);
                        } else {
                            for (Robot r : robots) {
                                r.updateKnownGrid(nextStep.x, nextStep.y, 1);
                            }
                        }
                    }
                    allAtRendezvous = false;
                }
            }

            long stepEndTime = System.nanoTime(); // End timing for this step
            System.out.println("Step Execution Time (ms): " + (stepEndTime - stepStartTime) / 1e6);

            if (gui != null) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        long totalEndTime = System.nanoTime(); // End timing the entire simulation
        System.out.println("Total Simulation Execution Time (ms): " + (totalEndTime - totalStartTime) / 1e6);
    }
}
