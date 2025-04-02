import java.io.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

class Point {
    int x, y;

    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

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

class Robot {
    Point position;
    int[][] knownGrid;
    List<Robot> peers;
    Color color;
    List<Point> currentPath;

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

    public void setPeers(List<Robot> peers) {
        this.peers = peers;
    }

    public void communicateObstacleData(int x, int y) {
        for (Robot robot : peers) {
            if (robot != this) {
                robot.updateKnownGrid(x, y, 1);
            }
        }
    }

    public void updateKnownGrid(int x, int y, int value) {
        knownGrid[x][y] = value;
    }

    public void move(Point nextStep) {
        if (isValidMove(nextStep)) {
            position = nextStep;
        } else {
            for (Robot r : peers) {
                r.updateKnownGrid(nextStep.x, nextStep.y, 1);
            }
        }
    }

    public boolean canMoveTo(Point nextStep) {
        return isValidMove(nextStep);
    }

    private boolean isValidMove(Point p) {
        if (p.x < 0 || p.x >= knownGrid.length || p.y < 0 || p.y >= knownGrid[0].length) {
            return false;
        }
        return knownGrid[p.x][p.y] == 0;
    }
}

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
        double g;
        double h;
        double f;

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

    public double manhattanHeuristic(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

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

    private List<Point> reconstructPath(Node node) {
        List<Point> path = new ArrayList<>();
        while (node != null) {
            path.add(0, node.point);
            node = node.parent;
        }
        return path;
    }
}

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
