package frc.robot.Drive.AutoAlign.Geometry;

/** Axis-aligned field-coordinate bounding box. */
public record Bounds(
    double minX,
    double maxX,
    double minY,
    double maxY
) {
    public Bounds {
        if (minX > maxX || minY > maxY) {
            throw new IllegalArgumentException("Invalid bounds");
        }
    }

    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    public Bounds expand(double amountMeters) {
        return new Bounds(
            minX - amountMeters,
            maxX + amountMeters,
            minY - amountMeters,
            maxY + amountMeters
        );
    }
}