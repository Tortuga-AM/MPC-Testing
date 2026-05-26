package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Zone {
    // Represents a rectangular zone on the field for auto-alignment purposes
    private final double xMin, xMax, yMin, yMax;
    private final String name;
    
    public Zone(double xMin, double xMax, double yMin, double yMax, String name) {
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
        this.name = name;
    }

    public boolean contains(double x, double y) {
        return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
    }
    public boolean contains(Translation2d point) {
        return contains(point.getX(), point.getY());
    }
    public boolean contains(Pose2d pose) {
        return contains(pose.getX(), pose.getY());
    }

    public String getName() {
        return name;
    }
    public double getXMin() { return xMin; }
    public double getXMax() { return xMax; }
    public double getYMin() { return yMin; }
    public double getYMax() { return yMax; }
    public Translation2d getCenter() {
        return new Translation2d((xMin + xMax) / 2, (yMin + yMax) / 2);
    }
    public Pose2d[] getCorners() {
        return new Pose2d[] {
            new Pose2d(xMin, yMin, null),
            new Pose2d(xMin, yMax, null),
            new Pose2d(xMax, yMin, null),
            new Pose2d(xMax, yMax, null),
            new Pose2d(xMin, yMin, null), // Bring back the first corner for logging
        };
    }
}