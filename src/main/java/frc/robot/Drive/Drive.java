package frc.robot.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Drive.DriveConstants.ANGULAR_VELOCITY_LIMIT;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.Drive.TrajoPlotPro.RobotConfig;

import static frc.robot.GlobalConstants.Controllers.*;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class Drive extends Subsystem<DriveStates> {
    private static Drive instance;

	private DriveIO driveIO;
    private boolean mpcAvailable = false;
    // private final LocalNonLinearProblemSolver solver = new LocalNonLinearProblemSolver();
    private final RobotConfig config = new RobotConfig(
            135.0, // massLbs
            6.0,  // moi
            2.0,   // wheelRadiusInches
            1.5,   // wheelCOF
            6.5,  // gearRatio
            6000.0, // maxMotorRPM
            1.2,  // maxMotorTorqueNm
            new Translation2d[] { new Translation2d(0.2794, 0.2794), new Translation2d(-0.2794, 0.2794), new Translation2d(-0.2794, -0.2794), new Translation2d(0.2794, -0.2794) }, // moduleOffsets
            4.3,   // maxVelocityOverride
            7.5    // maxAccelOverride
    );
    private final TrajoPlotPro solver = new TrajoPlotPro(
        config
    );
    private boolean trajGen = false;
    private Timer pathFollowingTimer = new Timer();
    private double lastAccelTimestamp = 0.0;
    private ChassisSpeeds lastRobotSpeeds = new ChassisSpeeds();
    private PIDController xController = new PIDController(10, 0, 0);
    private PIDController yController = new PIDController(10, 0, 0);
    private PIDController headingController = new PIDController(15, 0, 0);
    private Trajectory<SwerveSample> pathfindingTrajectory = null;
    private boolean firstLoop = true;

    private Field2d field2d = new Field2d();
    // =========================================================================
    // Constructor
    // =========================================================================
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
        super("Drive", DriveStates.MANUAL);
        this.driveIO = switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new DriveIOReal();
			case SIM -> new DriveIOSim();
			case TESTING -> new DriveIOReal();
		};
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        try {
            TinyMPCJNI.initialize();
            mpcAvailable = true;
        } catch (Throwable t) {
            mpcAvailable = false;
            System.err.println("TinyMPC JNI unavailable; MPC follow will be disabled.");
            if (TinyMPCJNI.getLoadError() != null) {
                TinyMPCJNI.getLoadError().printStackTrace();
            } else {
                t.printStackTrace();
            }
        }
    }

    @Override
    public void runState() {
        field2d.setRobotPose(getPose());
        Pose2d pose = getPose();
        Logger.recordOutput("Field", pose);
        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putString("Drive State", getState().toString());
        SmartDashboard.putNumber("Drive State Time", getStateTime());
        if (DRIVER_CONTROLLER.getXButtonPressed()) {
            setPathfindingTarget(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
        }
        switch (getState()) {
            case MANUAL -> {
                firstLoop = true;
                // Get joystick inputs
                double xSpeed = -DRIVER_CONTROLLER.getLeftY(); // Forward/backward
                double ySpeed = -DRIVER_CONTROLLER.getLeftX(); // Left/right
                double rotSpeed = -DRIVER_CONTROLLER.getRightX(); // Rotation

                // Create chassis speeds from joystick inputs
                ChassisSpeeds speeds = new ChassisSpeeds(xSpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), ySpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), rotSpeed * ANGULAR_VELOCITY_LIMIT.in(DegreesPerSecond));
                drive(speeds);
            }
            case PATH_FOLLOWING_CENTER, PATHFINDING -> {
                if (!trajGen && (getVelocityX()+0.1 > config.maxVelocityOverride() || getVelocityY()+0.1 > config.maxVelocityOverride())) {
                    drive(new ChassisSpeeds());
                    System.err.println("Robot is moving too fast for path generation! Breaking to restore possibilities");
                    // Will reattempt next loop
                    return;
                }
                if (!trajGen) {
                    new ChassisSpeeds();
                    ChassisSpeeds currentRobotSpeeds = new ChassisSpeeds(
                        driveIO.getDrive().getState().Speeds.vxMetersPerSecond,
                        driveIO.getDrive().getState().Speeds.vyMetersPerSecond,
                        driveIO.getDrive().getState().Speeds.omegaRadiansPerSecond
                    );
                    ChassisSpeeds currentFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                        currentRobotSpeeds.vxMetersPerSecond,
                        currentRobotSpeeds.vyMetersPerSecond,
                        currentRobotSpeeds.omegaRadiansPerSecond,
                        getPose().getRotation()
                    );
                    ChassisSpeeds currentFieldAcceleration = getCurrentFieldAcceleration(currentRobotSpeeds);
                    List<TrajoPlotPro.TrajectoryPoint> trajectory = solver.solve(getPose(), currentFieldSpeeds, currentFieldAcceleration, getState().getPose(), new ArrayList<Pose2d>() {{ add(new Pose2d(6, 6, Rotation2d.kZero)); }});
                    Pose2d[] poses = solver.reconstructToPose2d(trajectory);
                    field2d.getObject("MPPI Trajectory").setPoses(poses);
                    Trajectory<SwerveSample> ChoreoTrajectory = solver.convertToChoreo(trajectory);
                    if (ChoreoTrajectory == null) {
                        System.err.println("Trajectory generation failed! Breaking to restore possibilities");
                        driveIO.setControl(new SwerveRequest.Idle());
                        // Will reattempt next loop
                        return;
                    }
                    // holonomicController.startPath(trajectory);
                    trajGen = true;
                    SetPath(ChoreoTrajectory);
                 }
                if (firstLoop) {
                    firstLoop = false;
                    Optional<SwerveSample> initialSample = pathfindingTrajectory.sampleAt(0.0, false);
                    ChassisSpeeds initialSpeeds = new ChassisSpeeds(initialSample.get().vx, initialSample.get().vy, initialSample.get().omega);
                    drive(initialSpeeds);
                    return;
                }
                // if ((pathfindingTrajectory.getInitialPose(false).get().relativeTo(getPose()).getX() > 0.1 || pathfindingTrajectory.getInitialPose(false).get().relativeTo(getPose()).getY() > 0.1 || Math.abs(pathfindingTrajectory.getInitialPose(false).get().relativeTo(getPose()).getRotation().getDegrees()) > 2.5 || driveIO.getDrive().getState().Speeds.vxMetersPerSecond > 0.1 || driveIO.getDrive().getState().Speeds.vyMetersPerSecond > 0.1 || Math.abs(driveIO.getDrive().getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(5)) && !ready) {
                //     drive(new ChassisSpeeds(xController.calculate(getPose().getX(), pathfindingTrajectory.getInitialPose(false).get().getX()), yController.calculate(getPose().getY(), pathfindingTrajectory.getInitialPose(false).get().getY()), headingController.calculate(getPose().getRotation().getRadians(), pathfindingTrajectory.getInitialPose(false).get().getRotation().getRadians())));
                // } else {
                //     ready = true;
                // }
                //if (ready) {
                    if (MPCFOllow()) {
                        trajGen = false;
                        setState(DriveStates.MANUAL);
                        System.out.println("Trajectory complete!");
                        System.out.println("Final Pose: " + getPose());
                        System.out.println("Final Pose relative to goal: " + getPose().relativeTo(pathfindingTrajectory.getFinalPose(false).get()));
                        System.out.println("Total Time: " + pathFollowingTimer.get() + " seconds");
                        pathFollowingTimer.stop();
                        pathFollowingTimer.reset();
                    }
                    //MPCFOllow();
                //}
            }
            default -> {
                 driveIO.setControl(new SwerveRequest.SwerveDriveBrake());
            }
        }
    }
    // Field Centric drive with discretization and desaturation
    protected void drive(ChassisSpeeds speeds) {
        //speeds = ChassisSpeeds.discretize(speeds, 0.02);
        driveIO.setControl(new SwerveRequest.FieldCentric().withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }
    private boolean MPCFOllow() {
        if (!mpcAvailable) {
            return false;
        }
        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(driveIO.getDrive().getState().Speeds, getPose().getRotation());
        double[] currentState = {
            getPose().getX(),
            getPose().getY(),
            getPose().getRotation().getRadians(),
            currentSpeeds.vxMetersPerSecond,
            currentSpeeds.vyMetersPerSecond,
            currentSpeeds.omegaRadiansPerSecond
        };
        Optional<SwerveSample> sample0 = pathfindingTrajectory.sampleAt(pathFollowingTimer.get(), false);
        // Make this a loop to avoid copy-paste?
        SwerveSample[] samples = new SwerveSample[30];
        for (int i = 0; i < 30; i++) {
            samples[i] = pathfindingTrajectory.sampleAt(pathFollowingTimer.get() + (i * 0.02), false).get();
        }

        double[][] referenceHorizon = new double[30][6];
        for (int i = 0; i < 10; i++) {
            referenceHorizon[i][0] = samples[i].x; // Target X
            referenceHorizon[i][1] = samples[i].y; // Target Y
            referenceHorizon[i][2] = samples[i].heading; // Target Theta
            referenceHorizon[i][3] = samples[i].vx; // Target vx
            referenceHorizon[i][4] = samples[i].vy; // Target vy
            referenceHorizon[i][5] = samples[i].omega; // Target omega
        }
        // 2. Get the acceleration recommended by the MPC
        //System.out.println(Arrays.toString(referenceHorizon[0]));
        double[] mpcAccel = TinyMPCJNI.getControl(currentState, referenceHorizon);
        double ax = mpcAccel[0];
        double ay = mpcAccel[1];
        double alpha = mpcAccel[2];
        SmartDashboard.putNumber("MPC Accel X", ax);
        SmartDashboard.putNumber("MPC Accel Y", ay);
        SmartDashboard.putNumber("MPC Accel Omega", alpha);

        // 3. Calculate the "Next Best Velocity"
        // We multiply by 0.02 because that is our loop time (dt)
        double nextVx = currentSpeeds.vxMetersPerSecond + (ax * 0.02);
        double nextVy = currentSpeeds.vyMetersPerSecond + (ay * 0.02);
        double nextOmega = currentSpeeds.omegaRadiansPerSecond + (alpha * 0.02);
        SmartDashboard.putNumber("Next Vx", nextVx);
        SmartDashboard.putNumber("Next Vy", nextVy);
        SmartDashboard.putNumber("Next Omega", nextOmega);
        SmartDashboard.putNumber("Sample Vx", sample0.get().getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Sample Vy", sample0.get().getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Sample Omega", sample0.get().getChassisSpeeds().omegaRadiansPerSecond);
        // 4. Command the robot
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(nextVx, nextVy, nextOmega);
        drive(targetSpeeds);
        field2d.getObject("Ghost").setPose(sample0.get().getPose());
        if (getPose().getMeasureX().in(Meters) - pathfindingTrajectory.getFinalPose(false).get().getX() < 0.1 && getPose().getMeasureY().in(Meters) - pathfindingTrajectory.getFinalPose(false).get().getY() < 0.1 && Math.abs(getPose().getRotation().relativeTo(pathfindingTrajectory.getFinalPose(false).get().getRotation()).getRadians()) < Math.toRadians(1.0) && pathFollowingTimer.hasElapsed(pathfindingTrajectory.getTotalTime()))  {
            drive(new ChassisSpeeds());
            trajGen = false;
            setState(DriveStates.MANUAL);
            return true; // Trajectory complete
        }
        return false; // Trajectory not yet complete

    }
    private void SetPath(Trajectory<SwerveSample> trajectory) {
        pathfindingTrajectory = trajectory;
        pathFollowingTimer.stop();
        pathFollowingTimer.reset();
        pathFollowingTimer.start();
    }
    private boolean followTrajectory() {
        Pose2d pose = getPose();
        Optional<SwerveSample> sample = pathfindingTrajectory.sampleAt(pathFollowingTimer.get(), false);
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.get().vx + xController.calculate(pose.getX(), sample.get().x),
            sample.get().vy + yController.calculate(pose.getY(), sample.get().y),
            sample.get().omega + headingController.calculate(pose.getRotation().getRadians(), sample.get().heading)
        );
        field2d.getObject("Ghost").setPose(sample.get().getPose());
        drive(speeds);
        if (getPose().getMeasureX().in(Meters) - pathfindingTrajectory.getFinalPose(false).get().getX() < 0.1 && getPose().getMeasureY().in(Meters) - pathfindingTrajectory.getFinalPose(false).get().getY() < 0.1 && Math.abs(getPose().getRotation().relativeTo(pathfindingTrajectory.getFinalPose(false).get().getRotation()).getRadians()) < Math.toRadians(1.0) && pathFollowingTimer.hasElapsed(pathfindingTrajectory.getTotalTime()))  {
            drive(new ChassisSpeeds());
            trajGen = false;
            setState(DriveStates.MANUAL);
            return true; // Trajectory complete
        }
        return false; // Trajectory not yet complete
    }


    public void putTrajectory(String key, Pose2d[] trajectory) {
        field2d.getObject(key).setPoses(trajectory);
    }

    // Robot Centric drive with discretization and desaturation
    protected void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        driveIO.setControl(new SwerveRequest.RobotCentric().withDesaturateWheelSpeeds(true).withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    /**
     * Get the robot's current field-relative pose from your pose estimator.
     */
    protected Pose2d getPose() {
        return driveIO.getDrive().getState().Pose;
    }

    /** Chassis-relative X velocity (m/s). Implement from your actual odometry. */
    protected double getVelocityX() { return driveIO.getDrive().getState().Speeds.vxMetersPerSecond; }
    /** Chassis-relative Y velocity (m/s). */
    protected double getVelocityY() { return driveIO.getDrive().getState().Speeds.vyMetersPerSecond; }
    /** Angular velocity (rad/s). */
    protected double getAngularVelocity() { return driveIO.getDrive().getState().Speeds.omegaRadiansPerSecond; }

    private ChassisSpeeds getCurrentFieldAcceleration(ChassisSpeeds currentRobotSpeeds) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastAccelTimestamp;
        ChassisSpeeds accelRobot = new ChassisSpeeds();
        if (dt > 1e-3) {
            accelRobot = new ChassisSpeeds(
                (currentRobotSpeeds.vxMetersPerSecond - lastRobotSpeeds.vxMetersPerSecond) / dt,
                (currentRobotSpeeds.vyMetersPerSecond - lastRobotSpeeds.vyMetersPerSecond) / dt,
                (currentRobotSpeeds.omegaRadiansPerSecond - lastRobotSpeeds.omegaRadiansPerSecond) / dt
            );
        }
        lastRobotSpeeds = currentRobotSpeeds;
        lastAccelTimestamp = now;
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            accelRobot.vxMetersPerSecond,
            accelRobot.vyMetersPerSecond,
            accelRobot.omegaRadiansPerSecond,
            getPose().getRotation()
        );
    }

    /** Convenience helper for setting a one-off pathfinding target. */
    public void setPathfindingTarget(Pose2d targetPose) {
        DriveStates.PATHFINDING.setPose(targetPose);
        setState(DriveStates.PATHFINDING);
    }

}