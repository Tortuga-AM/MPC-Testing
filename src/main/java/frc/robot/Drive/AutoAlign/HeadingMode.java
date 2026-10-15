package frc.robot.Drive.AutoAlign;

/** Determines how desired robot heading is generated along a trajectory. */
public enum HeadingMode {
    /**
     * Current behavior: use a trapezoidal angular profile from start heading
     * to end heading.
     */
    PROFILED,

    /**
     * Interpolate heading from start heading to final heading based on path
     * distance traveled.
     */
    INTERPOLATE,

    /**
     * Point the robot in the direction of translational travel.
     */
    PATH_TANGENT,

    /**
     * Ask a user-provided HeadingFunction for the heading at each trajectory
     * sample.
     */
    FUNCTION
}