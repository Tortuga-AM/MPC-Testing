package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicPoseController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final double maxLinearSpeedMetersPerSecond;
    private final double maxAngularSpeedRadiansPerSecond;

    public HolonomicPoseController(
        double translationKP,
        double translationKI,
        double translationKD,
        double rotationKP,
        double rotationKI,
        double rotationKD,
        double maxLinearSpeedMetersPerSecond,
        double maxAngularSpeedRadiansPerSecond
    ) {
        xController = new PIDController(
            translationKP,
            translationKI,
            translationKD
        );

        yController = new PIDController(
            translationKP,
            translationKI,
            translationKD
        );

        rotationController = new PIDController(
            rotationKP,
            rotationKI,
            rotationKD
        );

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.maxLinearSpeedMetersPerSecond = maxLinearSpeedMetersPerSecond;
        this.maxAngularSpeedRadiansPerSecond =
            maxAngularSpeedRadiansPerSecond;
    }

    public void reset() {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    /**
     * Calculate a robot-relative command to approach a stationary target pose.
     *
     * @param currentPose Current estimated field pose
     * @param targetPose Desired field pose
     * @return Robot-relative chassis-speed command
     */
    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d targetPose,
        Translation2d fieldVelocityFeedforward,
        double omegaFeedforwardRadPerSecond
    ) {
        double fieldVx =
            fieldVelocityFeedforward.getX()
                + xController.calculate(currentPose.getX(), targetPose.getX());

        double fieldVy =
            fieldVelocityFeedforward.getY()
                + yController.calculate(currentPose.getY(), targetPose.getY());

        double omega =
            omegaFeedforwardRadPerSecond
                + rotationController.calculate(
                    currentPose.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

        Translation2d translationCommand = new Translation2d(fieldVx, fieldVy);

        if (translationCommand.getNorm() > maxLinearSpeedMetersPerSecond) {
            translationCommand = translationCommand.times(
                maxLinearSpeedMetersPerSecond / translationCommand.getNorm()
            );
        }

        omega = MathUtil.clamp(
            omega,
            -maxAngularSpeedRadiansPerSecond,
            maxAngularSpeedRadiansPerSecond
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            translationCommand.getX(),
            translationCommand.getY(),
            omega,
            currentPose.getRotation()
        );
    }

    public boolean atTarget(
        Pose2d currentPose,
        Pose2d targetPose,
        double positionToleranceMeters,
        double headingToleranceRadians
    ) {
        double positionError = currentPose.getTranslation().getDistance(
            targetPose.getTranslation()
        );

        double headingError = Math.abs(
            MathUtil.angleModulus(
                targetPose.getRotation().getRadians()
                    - currentPose.getRotation().getRadians()
            )
        );

        return positionError <= positionToleranceMeters
            && headingError <= headingToleranceRadians;
    }
}