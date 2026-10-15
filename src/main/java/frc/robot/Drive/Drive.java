package frc.robot.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Drive.DriveConstants.ANGULAR_VELOCITY_LIMIT;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.GlobalConstants;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {
    private static Drive instance;

    private final DriveIO driveIO;
    private final Field2d field2d = new Field2d();

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }

        return instance;
    }

    private Drive() {
        super("Drive", DriveStates.MANUAL);

        driveIO = switch (GlobalConstants.ROBOT_MODE) {
            case REAL, TESTING -> new DriveIOReal();
            case SIM -> new DriveIOSim();
        };
    }

    @Override
    protected void runState() {
        switch (getState()) {
            case MANUAL -> runManualDrive();

            case AUTO_ALIGN -> {
                /*
                 * AutoAlign commands the drivetrain using driveRobotRelative().
                 *
                 * Do not put a drive command here. If you do, Drive.periodic()
                 * will overwrite AutoAlign's command every scheduler cycle.
                 */
            }

            case BRAKE -> driveIO.setControl(new SwerveRequest.SwerveDriveBrake());
        }

        updateTelemetry();
    }

    private void runManualDrive() {
        double xInput = -DRIVER_CONTROLLER.getLeftY();
        double yInput = -DRIVER_CONTROLLER.getLeftX();
        double rotationInput = -DRIVER_CONTROLLER.getRightX();

        ChassisSpeeds speeds = new ChassisSpeeds(
            xInput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            yInput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),

            /*
             * This must be radians/sec.
             *
             * Your original code used DegreesPerSecond, but:
             * - ChassisSpeeds.omegaRadiansPerSecond is radians/sec
             * - CTRE withRotationalRate() expects radians/sec
             */
            rotationInput * ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond)
        );

        driveFieldRelative(speeds);
    }

    /**
     * Commands field-relative chassis speeds.
     *
     * Intended primarily for manual driver control.
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveIO.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Commands robot-relative chassis speeds.
     *
     * AutoAlign, a future trajectory follower, and any custom controller should
     * use this method.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        driveIO.setControl(
            new SwerveRequest.RobotCentric()
                .withDesaturateWheelSpeeds(true)
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /** Command zero chassis velocity. */
    public void stop() {
        driveRobotRelative(new ChassisSpeeds());
    }

    /** Put the drivetrain into CTRE brake/X-lock behavior. */
    public void brake() {
        setState(DriveStates.BRAKE);
    }

    public void resetPose(Pose2d pose) {
        driveIO.getDrive().resetPose(pose);
    }

    public Pose2d getPose() {
        return driveIO.getDrive().getState().Pose;
    }

    /**
     * Returns measured robot-relative chassis speeds.
     *
     * CTRE's SwerveDrivetrain state Speeds are robot-relative.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return driveIO.getDrive().getState().Speeds;
    }

    public void putTrajectory(String key, Pose2d[] trajectory) {
        field2d.getObject(key).setPoses(trajectory);
    }

    private void updateTelemetry() {
        field2d.setRobotPose(getPose());

        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/State", getState().getStateString());
        Logger.recordOutput(
            "Drive/RobotRelativeSpeeds",
            getRobotRelativeSpeeds()
        );

        Logger.recordOutput(
            "Drive/ModuleStates",
            getModuleStates()
        );

        SmartDashboard.putData("Drive/Field", field2d);
        SmartDashboard.putString("Drive/State", getState().getStateString());
        SmartDashboard.putNumber("Drive/StateTimeSeconds", getStateTime());
    }

    /**
     * Returns measured module states from the drivetrain.
     *
     * <p>These are useful for:
     * <ul>
     *   <li>logging</li>
     *   <li>module acceleration limiting</li>
     *   <li>steering-rate limiting</li>
     *   <li>future custom setpoint generation</li>
     * </ul>
     */
    public SwerveModuleState[] getModuleStates() {
        return driveIO.getModuleStates();
    }

    /**
     * Commands desired module states directly.
     *
     * <p>Use this only when your custom setpoint generator owns module-level
     * control. For normal chassis-level requests, use driveRobotRelative().
     */
    public void setModuleStates(SwerveModuleState[] states) {
        driveIO.setModuleStates(states);
    }
}