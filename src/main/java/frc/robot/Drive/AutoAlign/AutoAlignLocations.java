package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public enum AutoAlignLocations {
    CENTER(new Pose2d(8.283, 4.115, Rotation2d.fromDegrees(180.0))),
    HUB(new Pose2d(4.25, 4.115, Rotation2d.fromDegrees(0.0))),
    REFILL(new Pose2d(1, 1, Rotation2d.fromDegrees(180.0)));

    private Pose2d pose;

    private AutoAlignLocations(Pose2d pose) {
        this.pose = pose;
    }

    public Pose2d getPose() {
        if (Robot.isBlue) {
            return new Pose2d(
                16.54 - pose.getX(),
                8.23 - pose.getY(),
                pose.getRotation().plus(Rotation2d.fromDegrees(180.0))
            );
        }
        return pose;
    }
}
