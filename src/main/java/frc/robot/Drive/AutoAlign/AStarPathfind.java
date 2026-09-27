package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Drive.AutoAlign.TrajoPlotPro.ConstraintCirc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

public class AStarPathfind {

    private static final double RESOLUTION = 0.15; // Kept large for rapid solves
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;
    private static final int GRID_WIDTH = (int) Math.ceil(FIELD_LENGTH / RESOLUTION);
    private static final int GRID_HEIGHT = (int) Math.ceil(FIELD_WIDTH / RESOLUTION);

    private static class Node implements Comparable<Node> {
        int x, y;
        double gCost = Double.MAX_VALUE;
        double hCost;
        Node parent;

        Node(int x, int y) {
            this.x = x;
            this.y = y;
        }

        double fCost() { return gCost + hCost; }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fCost(), other.fCost());
        }
    }

    public static List<Translation2d> findPath(Translation2d start, Translation2d goal, List<ConstraintCirc> env) {
        int startX = clampGridX((int) (start.getX() / RESOLUTION));
        int startY = clampGridY((int) (start.getY() / RESOLUTION));
        int goalX = clampGridX((int) (goal.getX() / RESOLUTION));
        int goalY = clampGridY((int) (goal.getY() / RESOLUTION));

        PriorityQueue<Node> openSet = new PriorityQueue<>();
        boolean[][] closedSet = new boolean[GRID_WIDTH][GRID_HEIGHT];
        Node[][] nodes = new Node[GRID_WIDTH][GRID_HEIGHT];

        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                nodes[x][y] = new Node(x, y);
            }
        }

        Node startNode = nodes[startX][startY];
        startNode.gCost = 0;
        startNode.hCost = getHeuristic(startX, startY, goalX, goalY);
        openSet.add(startNode);

        Node targetNode = null;
        int[][] directions = {{0,1}, {1,0}, {0,-1}, {-1,0}, {1,1}, {1,-1}, {-1,1}, {-1,-1}};

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            closedSet[current.x][current.y] = true;

            if (current.x == goalX && current.y == goalY) {
                targetNode = current;
                break;
            }

            for (int[] dir : directions) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];

                if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT) continue;
                if (closedSet[nx][ny]) continue;
                if (!isWalkable(nx, ny, env)) continue;

                Node neighbor = nodes[nx][ny];
                
                if (current.parent != null && hasLineOfSight(current.parent.x, current.parent.y, nx, ny, env)) {
                    double costToNeighbor = current.parent.gCost + getHeuristic(current.parent.x, current.parent.y, nx, ny);
                    if (costToNeighbor < neighbor.gCost) {
                        neighbor.gCost = costToNeighbor;
                        neighbor.hCost = getHeuristic(nx, ny, goalX, goalY);
                        neighbor.parent = current.parent; 
                        if (!openSet.contains(neighbor)) openSet.add(neighbor);
                    }
                } else {
                    double moveCost = (dir[0] != 0 && dir[1] != 0) ? 1.414 : 1.0;
                    double tentativeGCost = current.gCost + moveCost;
                    if (tentativeGCost < neighbor.gCost) {
                        neighbor.gCost = tentativeGCost;
                        neighbor.hCost = getHeuristic(nx, ny, goalX, goalY);
                        neighbor.parent = current;
                        if (!openSet.contains(neighbor)) openSet.add(neighbor);
                    }
                }
            }
        }

        if (targetNode == null) return List.of();

        List<Translation2d> rawPath = new ArrayList<>();
        Node curr = targetNode;
        while (curr != null) {
            rawPath.add(new Translation2d(curr.x * RESOLUTION, curr.y * RESOLUTION));
            curr = curr.parent;
        }
        Collections.reverse(rawPath);

        rawPath.set(0, start);
        rawPath.set(rawPath.size() - 1, goal);

        return rawPath; 
    }

    private static boolean isWalkable(int gx, int gy, List<ConstraintCirc> env) {
        double px = gx * RESOLUTION;
        double py = gy * RESOLUTION;

        if (px < 0.1 || px > FIELD_LENGTH - 0.1 || py < 0.1 || py > FIELD_WIDTH - 0.1) return false;

        for (ConstraintCirc obs : env) {
            double dx = px - obs.x();
            double dy = py - obs.y();
            if (dx * dx + dy * dy <= obs.rSq()) return false;
        }
        return true;
    }

    private static boolean hasLineOfSight(int x1, int y1, int x2, int y2, List<ConstraintCirc> env) {
        if (x1 == x2 && y1 == y2) return true;
        
        double px1 = x1 * RESOLUTION; double py1 = y1 * RESOLUTION;
        double px2 = x2 * RESOLUTION; double py2 = y2 * RESOLUTION;
        
        double dist = Math.hypot(px2 - px1, py2 - py1);
        int steps = Math.max(1, (int) Math.ceil(dist / (RESOLUTION / 2.0))); 
        
        for (int i = 0; i <= steps; i++) {
            double t = (double) i / steps;
            double ptX = px1 + (px2 - px1) * t;
            double ptY = py1 + (py2 - py1) * t;
            
            if (ptX < 0.1 || ptX > FIELD_LENGTH - 0.1 || ptY < 0.1 || ptY > FIELD_WIDTH - 0.1) return false;
            
            for (ConstraintCirc obs : env) {
                double dx = ptX - obs.x();
                double dy = ptY - obs.y();
                if (dx * dx + dy * dy <= obs.rSq()) return false;
            }
        }
        return true;
    }

    private static double getHeuristic(int x1, int y1, int x2, int y2) { return Math.hypot(x1 - x2, y1 - y2); }
    private static int clampGridX(int val) { return Math.max(0, Math.min(GRID_WIDTH - 1, val)); }
    private static int clampGridY(int val) { return Math.max(0, Math.min(GRID_HEIGHT - 1, val)); }
}
