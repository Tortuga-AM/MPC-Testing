package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

import frc.robot.Drive.AutoAlign.TrajoPlotPro.RectObstacle;

public class AStarPathfind {

    private static final double RESOLUTION = 0.10; // Increased to 20cm for faster grid generation
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

    public static List<Translation2d> findPath(Translation2d start, Translation2d goal, 
                                               List<Translation2d> circularObstacles, 
                                               List<RectObstacle> rectObstacles, 
                                               double avoidanceRadius) {
        
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
                if (!isWalkable(nx, ny, circularObstacles, rectObstacles, avoidanceRadius)) continue;

                Node neighbor = nodes[nx][ny];
                
                // THETA* ANY-ANGLE LOGIC: Check Line of Sight to Grandparent
                if (current.parent != null && hasLineOfSight(current.parent.x, current.parent.y, nx, ny, circularObstacles, rectObstacles, avoidanceRadius)) {
                    double costToNeighbor = current.parent.gCost + getHeuristic(current.parent.x, current.parent.y, nx, ny);
                    if (costToNeighbor < neighbor.gCost) {
                        neighbor.gCost = costToNeighbor;
                        neighbor.hCost = getHeuristic(nx, ny, goalX, goalY);
                        neighbor.parent = current.parent; // Bypass the current node!
                        if (!openSet.contains(neighbor)) openSet.add(neighbor);
                    }
                } else {
                    // Standard A* Step
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

        if (targetNode == null) return new ArrayList<>(List.of(start, goal));

        List<Translation2d> rawPath = new ArrayList<>();
        Node curr = targetNode;
        while (curr != null) {
            rawPath.add(new Translation2d(curr.x * RESOLUTION, curr.y * RESOLUTION));
            curr = curr.parent;
        }
        Collections.reverse(rawPath);

        rawPath.set(0, start);
        rawPath.set(rawPath.size() - 1, goal);

        return rawPath; // No smoothing pass needed, Theta* handles it natively!
    }

    private static boolean isWalkable(int gx, int gy, List<Translation2d> circs, List<RectObstacle> rects, double radius) {
        double px = gx * RESOLUTION;
        double py = gy * RESOLUTION;

        if (px < radius || px > FIELD_LENGTH - radius || py < radius || py > FIELD_WIDTH - radius) return false;

        if (circs != null) {
            for (Translation2d obs : circs) {
                if (Math.hypot(px - obs.getX(), py - obs.getY()) < radius) return false;
            }
        }
        if (rects != null) {
            for (RectObstacle rect : rects) {
                double hW = (rect.width() / 2.0) + radius;
                double hH = (rect.height() / 2.0) + radius;
                if (px > rect.x() - hW && px < rect.x() + hW && py > rect.y() - hH && py < rect.y() + hH) return false;
            }
        }
        return true;
    }

    private static boolean hasLineOfSight(int x1, int y1, int x2, int y2, List<Translation2d> circs, List<RectObstacle> rects, double radius) {
        double px1 = x1 * RESOLUTION; double py1 = y1 * RESOLUTION;
        double px2 = x2 * RESOLUTION; double py2 = y2 * RESOLUTION;
        
        double dist = Math.hypot(px2 - px1, py2 - py1);
        int steps = (int) Math.ceil(dist / (RESOLUTION / 2.0)); 
        
        for (int i = 0; i <= steps; i++) {
            double t = (double) i / steps;
            double ptX = px1 + (px2 - px1) * t;
            double ptY = py1 + (py2 - py1) * t;
            
            if (ptX < radius || ptX > FIELD_LENGTH - radius || ptY < radius || ptY > FIELD_WIDTH - radius) return false;
            
            if (circs != null) {
                for (Translation2d obs : circs) {
                    if (Math.hypot(ptX - obs.getX(), ptY - obs.getY()) < radius) return false;
                }
            }
            if (rects != null) {
                for (RectObstacle rect : rects) {
                    double hW = (rect.width() / 2.0) + radius;
                    double hH = (rect.height() / 2.0) + radius;
                    if (ptX > rect.x() - hW && ptX < rect.x() + hW && ptY > rect.y() - hH && ptY < rect.y() + hH) return false;
                }
            }
        }
        return true;
    }

    private static double getHeuristic(int x1, int y1, int x2, int y2) { return Math.hypot(x1 - x2, y1 - y2); }
    private static int clampGridX(int val) { return Math.max(0, Math.min(GRID_WIDTH - 1, val)); }
    private static int clampGridY(int val) { return Math.max(0, Math.min(GRID_HEIGHT - 1, val)); }
}