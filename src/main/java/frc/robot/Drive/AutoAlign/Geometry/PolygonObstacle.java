package frc.robot.Drive.AutoAlign.Geometry;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Arbitrary simple polygon obstacle using ray-casting containment.
 *
 * <p>Vertices must describe a non-self-intersecting polygon boundary.
 */
public class PolygonObstacle implements Obstacle {
    private final List<Translation2d> vertices;
    private final Bounds bounds;

    public PolygonObstacle(List<Translation2d> vertices, double inflationRadius) {
        this(inflateVertices(vertices, inflationRadius));
    }

    /**
     * Inflates a polygon's vertices outward by the given radius.
     */
    private static List<Translation2d> inflateVertices(List<Translation2d> vertices, double radius) {
        if (vertices == null || vertices.size() < 3 || radius == 0.0) {
            return vertices;
        }

        // 1. Determine winding order (Shoelace formula)
        // A negative sum indicates a Counter-Clockwise (CCW) polygon in standard Cartesian coordinates.
        double areaSum = 0;
        for (int i = 0; i < vertices.size(); i++) {
            Translation2d current = vertices.get(i);
            Translation2d next = vertices.get((i + 1) % vertices.size());
            areaSum += (next.getX() - current.getX()) * (next.getY() + current.getY());
        }
        boolean isCCW = areaSum < 0;

        List<Translation2d> inflated = new ArrayList<>();
        int n = vertices.size();

        for (int i = 0; i < n; i++) {
            Translation2d prev = vertices.get((i - 1 + n) % n);
            Translation2d curr = vertices.get(i);
            Translation2d next = vertices.get((i + 1) % n);

            // Normalized edge direction vectors entering and leaving the current vertex
            double inDx = curr.getX() - prev.getX();
            double inDy = curr.getY() - prev.getY();
            double inLen = Math.hypot(inDx, inDy);
            inDx /= inLen;
            inDy /= inLen;

            double outDx = next.getX() - curr.getX();
            double outDy = next.getY() - curr.getY();
            double outLen = Math.hypot(outDx, outDy);
            outDx /= outLen;
            outDy /= outLen;

            // Calculate the outward-facing normals based on winding order
            double nx1 = isCCW ? inDy : -inDy;
            double ny1 = isCCW ? -inDx : inDx;
            double nx2 = isCCW ? outDy : -outDy;
            double ny2 = isCCW ? -outDx : outDx;

            // Shift current vertex outwards along both normals
            double p1x = curr.getX() + nx1 * radius;
            double p1y = curr.getY() + ny1 * radius;

            double p2x = curr.getX() + nx2 * radius;
            double p2y = curr.getY() + ny2 * radius;

            // Find the intersection of the two shifted edges
            double det = inDx * outDy - inDy * outDx;

            if (Math.abs(det) < 1e-6) {
                // Edges are parallel/collinear; straight extension
                inflated.add(new Translation2d(p1x, p1y));
            } else {
                // Line intersection formula to find the sharp inflated corner
                double t = ((p2x - p1x) * outDy - (p2y - p1y) * outDx) / det;
                inflated.add(new Translation2d(p1x + t * inDx, p1y + t * inDy));
            }
        }

        return inflated;
    }


    public PolygonObstacle(List<Translation2d> vertices) {
        if (vertices == null || vertices.size() < 3) {
            throw new IllegalArgumentException(
                "Polygon requires at least three vertices"
            );
        }

        this.vertices = List.copyOf(vertices);

        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (Translation2d vertex : vertices) {
            minX = Math.min(minX, vertex.getX());
            maxX = Math.max(maxX, vertex.getX());
            minY = Math.min(minY, vertex.getY());
            maxY = Math.max(maxY, vertex.getY());
        }

        bounds = new Bounds(minX, maxX, minY, maxY);
    }

    @Override
    public boolean contains(Translation2d point) {
        if (!bounds.contains(point.getX(), point.getY())) {
            return false;
        }

        boolean inside = false;
        int previous = vertices.size() - 1;

        for (int current = 0; current < vertices.size(); current++) {
            Translation2d currentVertex = vertices.get(current);
            Translation2d previousVertex = vertices.get(previous);

            boolean straddlesHorizontalRay =
                (currentVertex.getY() > point.getY())
                    != (previousVertex.getY() > point.getY());

            if (straddlesHorizontalRay) {
                double xAtRayIntersection =
                    (previousVertex.getX() - currentVertex.getX())
                        * (point.getY() - currentVertex.getY())
                        / (previousVertex.getY() - currentVertex.getY())
                        + currentVertex.getX();

                if (point.getX() < xAtRayIntersection) {
                    inside = !inside;
                }
            }

            previous = current;
        }

        return inside;
    }

    @Override
    public Bounds getBounds() {
        return bounds;
    }

    public List<Translation2d> getVertices() {
        return new ArrayList<>(vertices);
    }
}