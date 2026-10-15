package frc.robot.Drive.AutoAlign;

/**
 * Requested heading behavior for a generated trajectory.
 */
public record HeadingRequest(
    HeadingMode mode,
    HeadingFunction function
) {
    public HeadingRequest {
        if (mode == null) {
            throw new IllegalArgumentException("Heading mode cannot be null");
        }

        if (mode == HeadingMode.FUNCTION && function == null) {
            throw new IllegalArgumentException(
                "FUNCTION heading mode requires a HeadingFunction"
            );
        }
    }

    /** Existing PathPlanner-like independent angular trapezoid behavior. */
    public static HeadingRequest profiled() {
        return new HeadingRequest(HeadingMode.PROFILED, null);
    }

    /** Smoothly interpolate from start heading to final heading over path progress. */
    public static HeadingRequest interpolate() {
        return new HeadingRequest(HeadingMode.INTERPOLATE, null);
    }

    /** Point in the direction the path is traveling. */
    public static HeadingRequest pathTangent() {
        return new HeadingRequest(HeadingMode.PATH_TANGENT, null);
    }

    /** Determine heading from a user-defined function. */
    public static HeadingRequest function(HeadingFunction function) {
        return new HeadingRequest(HeadingMode.FUNCTION, function);
    }
}