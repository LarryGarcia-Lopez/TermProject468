import java.util.*;
import java.awt.Point;

public class AStarPathFinder {
    private int[][] grid;
    private int rows, cols;

    // Constructor accepts the grid from your environment
    public AStarPathFinder(int[][] grid) {
        this.grid = grid;
        this.rows = grid.length;
        this.cols = grid[0].length;
    }

    // Node class to represent a point in the search space
    private class Node implements Comparable<Node> {
        Point point;
        Node parent;
        double g; // Cost from start to this node
        double h; // Heuristic cost from this node to goal
        double f; // Total cost f = g + h

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

    // Default heuristic: Manhattan distance
    public double manhattanHeuristic(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }
    
    // Alternative heuristic: Euclidean distance
    public double euclideanHeuristic(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    // Find path from start to goal using the A* algorithm.
    // You can switch the heuristic function here if desired.
    public List<Point> findPath(Point start, Point goal) {
        long startTime = System.nanoTime(); // Start measuring execution time
        
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        boolean[][] closedSet = new boolean[rows][cols];

        Node startNode = new Node(start, null, 0, manhattanHeuristic(start, goal));
        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            // Goal check
            if (current.point.equals(goal)) {
                long endTime = System.nanoTime(); // End measuring execution time
                List<Point> path = reconstructPath(current);
                
                System.out.println("Execution Time (ms): " + (endTime - startTime) / 1e6); // Print execution time
                System.out.println("Path Length: " + path.size()); // Print path length
                System.out.println("Optimal Path: " + path); // Print the optimal path
                return path;
            }

            closedSet[current.point.x][current.point.y] = true;

            // Explore neighbors (up, down, left, right)
            int[][] directions = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
            for (int[] dir : directions) {
                int newX = current.point.x + dir[0];
                int newY = current.point.y + dir[1];

                // Boundary check
                if (newX < 0 || newX >= rows || newY < 0 || newY >= cols)
                    continue;
                // Obstacle check (assumes 1 represents an obstacle)
                if (grid[newX][newY] == 1)
                    continue;
                // Skip if already evaluated
                if (closedSet[newX][newY])
                    continue;

                Point neighborPoint = new Point(newX, newY);
                double tentativeG = current.g + 1; // Assumes uniform cost (adjust if needed)
                // Here you can choose to use the Euclidean heuristic if desired:
                double h = manhattanHeuristic(neighborPoint, goal); // or euclideanHeuristic(neighborPoint, goal)
                Node neighbor = new Node(neighborPoint, current, tentativeG, h);

                openSet.add(neighbor);
            }
        }
        // No path found
        System.out.println("No path found.");
        return new ArrayList<>();
    }

    // Reconstruct the path by following parent links from goal back to start
    private List<Point> reconstructPath(Node node) {
        List<Point> path = new ArrayList<>();
        while (node != null) {
            path.add(0, node.point);  // Insert at the beginning
            node = node.parent;
        }
        return path;
    }

    // Performance Metrics (Use this to measure scalability with increasing number of robots)
    public void evaluatePerformance(int numRobots, int complexity, long executionTime, int pathLength) {
        System.out.println("Evaluating performance...");
        System.out.println("Number of Robots: " + numRobots);
        System.out.println("Environment Complexity: " + complexity);
        System.out.println("Average Execution Time (s): " + executionTime / 1e9);
        System.out.println("Average Path Length: " + pathLength);
        
        // How often robots take redundant paths (efficiency of communication, redundant paths)
        double redundancyFactor = (numRobots * pathLength) / (double) (rows * cols);
        System.out.println("Redundancy Factor: " + redundancyFactor);
    }
}
