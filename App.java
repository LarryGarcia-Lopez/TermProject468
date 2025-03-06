import java.io.*;
import java.util.*;

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
}

class Robot {
    Point position;

    public Robot(int x, int y) {
        this.position = new Point(x, y);
    }
}

class PathPlanningEnvironment {
    private int rows, cols;
    private int[][] grid;
    private List<Robot> robots = new ArrayList<>();
    private Point rendezvousPoint;

    public void loadEnvironment(String filename) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(filename));
        String[] dimensions = br.readLine().split(" ");
        rows = Integer.parseInt(dimensions[0]);
        cols = Integer.parseInt(dimensions[1]);

        grid = new int[rows][cols];

        int numRobots = Integer.parseInt(br.readLine().trim());

        for (int i = 0; i < numRobots; i++) {
            String[] robotPos = br.readLine().split(" ");
            robots.add(new Robot(Integer.parseInt(robotPos[0]), Integer.parseInt(robotPos[1])));
        }

        String[] rendezvousPos = br.readLine().split(" ");
        rendezvousPoint = new Point(Integer.parseInt(rendezvousPos[0]), Integer.parseInt(rendezvousPos[1]));

        for (int i = 0; i < rows; i++) {
            String line = br.readLine();
            for (int j = 0; j < cols; j++) {
                grid[i][j] = Character.getNumericValue(line.charAt(j));
            }
        }
        br.close();
    }

    public void displayEnvironment() {
        System.out.println("Grid Environment:");
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                boolean isRobot = false;
                for (Robot r : robots) {
                    if (r.position.x == i && r.position.y == j) {
                        System.out.print("R ");
                        isRobot = true;
                        break;
                    }
                }
                if (!isRobot) {
                    if (rendezvousPoint.x == i && rendezvousPoint.y == j) {
                        System.out.print("X ");
                    } else {
                        System.out.print((grid[i][j] == 1 ? "# " : ". "));
                    }
                }
            }
            System.out.println();
        }
    }
}

public class App {
    public static void main(String[] args) {
        PathPlanningEnvironment env = new PathPlanningEnvironment();
        try {
            env.loadEnvironment("Environment.txt");
            env.displayEnvironment();
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
        }
    }
}
