package frc.robot.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Drive.DriveConstants.ANGULAR_VELOCITY_LIMIT;
import static frc.robot.GlobalConstants.Controllers.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.Drive.AutoAlign.SpatialTrajectorySampler;
import frc.robot.Drive.AutoAlign.TrajoPlotPro;
import frc.robot.Drive.AutoAlign.Zone;
import frc.robot.Drive.AutoAlign.TrajoPlotPro.RobotConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicReference;

public class Drive extends Subsystem<DriveStates> {
    private static Drive instance;

    // --- Hardware & Config ---
    private final DriveIO driveIO;
    private final RobotConfig config = new RobotConfig(
            135.0, 6.0, 2.0, 1.5, 6.5, 6000.0, 1.2, 
            new Translation2d[] { new Translation2d(0.2794, 0.2794), new Translation2d(-0.2794, 0.2794), new Translation2d(-0.2794, -0.2794), new Translation2d(0.2794, -0.2794) }, 
            4.3, 8
    );
    private final TrajoPlotPro solver = new TrajoPlotPro(config);
    private final Field2d field2d = new Field2d();

    // --- Pathing State ---
    private boolean isGenerating = false;
    private boolean isFollowingPath = false;
    private boolean isInitialAlignmentReady = false;
    private AtomicReference<Trajectory<SwerveSample>> pendingTrajectory = new AtomicReference<>(null);
    private Trajectory<SwerveSample> activeTrajectory = null;
    private SpatialTrajectorySampler spatialSampler;    
    
    // --- Dynamic Target Tracking ---
    private Translation2d pointOfInterest = null;

    // --- Controllers ---
    private final PIDController xController = new PIDController(10, 0, 0); // Kept for micro-path alignments
    private final PIDController yController = new PIDController(10, 0, 0); 
    
    // NEW DECOUPLED PATH CONTROLLERS
    private final PIDController crossTrackController = new PIDController(10.0, 0, 0); // Aggressive obstacle avoidance
    private final PIDController alongTrackController = new PIDController(5.0, 0, 0);  // Soft, smooth longitudinal tracking
    private final PIDController headingController = new PIDController(15, 0, 0);

    // --- Kinematic Memory ---
    private double lastAccelTimestamp = 0.0;
    private ChassisSpeeds lastRobotSpeeds = new ChassisSpeeds();
    private ChassisSpeeds filteredAcceleration = new ChassisSpeeds();

    // --- APF & Replanning Constraints ---
    private final List<Translation2d> staticObstacles = new ArrayList<>();
    private final List<TrajoPlotPro.RectObstacle> rectObstacles = new ArrayList<>();
    private final List<Pose2d> waypoints = new ArrayList<>();
    private double stuckStartTime = -1.0;
    private static final double STUCK_VELOCITY_THRESHOLD = 0.15; 
    private static final double STUCK_ERROR_THRESHOLD = 0.2; 
    private static final double KICK_TRIGGER_TIME = 0.5; 
    private static final double BAILOUT_TRIGGER_TIME = 1.5; 
    private static final double BAILOUT_ERROR_THRESHOLD = 1.5; 
    private static final Zone NeutralLeftZone = new Zone(4.7, 8.26, 4.12, 15.0, "Neutral Left");

    // =========================================================================
    // Constructor & Singleton
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
            case REAL, TESTING -> new DriveIOReal();
            case SIM -> new DriveIOSim();
        };
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        
        addObstacle(4.5, 5.5); 
        rectObstacles.add(new TrajoPlotPro.RectObstacle(4.7, 4.12, 1.2, 5));
    }

    // =========================================================================
    // Main State Machine
    // =========================================================================
    @Override
    public void runState() {
        updateTelemetry();

        if (DRIVER_CONTROLLER.getXButtonPressed()) {
            if (NeutralLeftZone.contains(getPose().getTranslation())) {
                setPathfindingTarget(new Pose2d(1.4, 4.9, Rotation2d.fromDegrees(180.0)));
                waypoints.clear();
                waypoints.add(new Pose2d(5.65, 7.52, Rotation2d.fromDegrees(180.0)));
                waypoints.add(new Pose2d(3.5, 7.52, Rotation2d.fromDegrees(180.0)));
            } else {
                setPathfindingTarget(new Pose2d(2, 2, Rotation2d.fromDegrees(0.0)));
                waypoints.clear();
            }
        }
        
        switch (getState()) {
            case MANUAL -> {
                double xSpeed = -DRIVER_CONTROLLER.getLeftY(); 
                double ySpeed = -DRIVER_CONTROLLER.getLeftX(); 
                double rotSpeed = -DRIVER_CONTROLLER.getRightX(); 

                ChassisSpeeds speeds = new ChassisSpeeds(
                    xSpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 
                    ySpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 
                    rotSpeed * ANGULAR_VELOCITY_LIMIT.in(DegreesPerSecond)
                );
                drive(speeds);
            }
            case PATH_FOLLOWING_CENTER, PATHFINDING -> {
                if (!isFollowingPath && !isGenerating && pendingTrajectory.get() == null) {
                    Pose2d goal = getState().getPose();
                    
                    if (getPose().getTranslation().getDistance(goal.getTranslation()) < 0.1 && 
                        Math.abs(getVelocityX()) < 0.1 && Math.abs(getVelocityY()) < 0.1) {
                        
                        drive(new ChassisSpeeds(
                            xController.calculate(getPose().getX(), goal.getX()), 
                            yController.calculate(getPose().getY(), goal.getY()), 
                            headingController.calculate(getPose().getRotation().getRadians(), goal.getRotation().getRadians())
                        ));
                        return; 
                    }

                    if (getVelocityX() + 0.1 > config.maxVelocityOverride() || getVelocityY() + 0.1 > config.maxVelocityOverride()) {
                        drive(new ChassisSpeeds());
                        System.err.println("Braking to stabilize before path generation...");
                        return;
                    }
                    generateGlobalTrajectory(goal);
                }

                if (isGenerating) {
                    drive(new ChassisSpeeds());
                    return; 
                }

                if (pendingTrajectory.get() != null) {
                    Trajectory<SwerveSample> newTraj = pendingTrajectory.getAndSet(null); 
                    if (newTraj != null) {
                        setPath(newTraj);
                    } else {
                        driveIO.setControl(new SwerveRequest.Idle());
                        return;
                    }
                }

                if (isFollowingPath) {
                    handleInitialAlignmentAndFollow();
                }
            }
            default -> driveIO.setControl(new SwerveRequest.SwerveDriveBrake());
        }
    }

    // =========================================================================
    // Core Path Following & Replanning
    // =========================================================================
    private void handleInitialAlignmentAndFollow() {
        if (followTrajectory()) {
            isFollowingPath = false;
            activeTrajectory = null; 
            setState(DriveStates.MANUAL);
            System.out.println("Trajectory complete!");
        }
    }

    private boolean followTrajectory() {
        Pose2d pose = getPose();        
        SwerveSample targetSample = spatialSampler.getLookaheadSample(pose, new Translation2d(getVelocityX(), getVelocityY()));
        double closestPathTime = spatialSampler.getCurrentPathTime();
        
        SmartDashboard.putNumber("Closest Path Time", closestPathTime);
        field2d.getObject("Target Pose").setPose(targetSample.getPose());

        if (checkTrajectoryComplete(closestPathTime)) {
            drive(new ChassisSpeeds());
            return true; 
        }

        double targetX = targetSample.x;
        double targetY = targetSample.y;
        double ffVx = targetSample.vx;
        double ffVy = targetSample.vy;
        double ffOmega = targetSample.omega;

        // 1. DYNAMIC POI TRACKING OVERRIDE
        double targetHeading = targetSample.heading;
        if (pointOfInterest != null) {
            targetHeading = Math.atan2(pointOfInterest.getY() - pose.getY(), pointOfInterest.getX() - pose.getX());
            ffOmega = 0.0; // Let the PID exclusively handle the aggressive tracking spin
        }

        double distanceError = Math.hypot(targetX - pose.getX(), targetY - pose.getY());
        double currentSpeed = Math.hypot(getVelocityX(), getVelocityY());

        // STALEMATE TRACKER & BAILOUT TRIGGER
        if (distanceError > STUCK_ERROR_THRESHOLD && currentSpeed < STUCK_VELOCITY_THRESHOLD) {
            if (stuckStartTime < 0) stuckStartTime = Timer.getFPGATimestamp();
        } else {
            stuckStartTime = -1.0; 
        }

        double timeStuck = (stuckStartTime > 0) ? (Timer.getFPGATimestamp() - stuckStartTime) : 0.0;

        if (distanceError > BAILOUT_ERROR_THRESHOLD || timeStuck > BAILOUT_TRIGGER_TIME) {
            System.err.println("GLOBAL REPLAN TRIGGERED!");
            stuckStartTime = -1.0;
            isFollowingPath = false;
            setPathfindingTarget(activeTrajectory.getFinalPose(false).get()); 
            return false;
        }

        // =====================================================================
        // 2. DECOUPLED ERROR ROTATION (Cross-Track vs Along-Track)
        // =====================================================================
        double pathAngle;
        if (Math.hypot(ffVx, ffVy) > 0.05) {
            pathAngle = Math.atan2(ffVy, ffVx); // Direction the trajectory wants us to move
        } else {
            pathAngle = Math.atan2(targetY - pose.getY(), targetX - pose.getX()); // Fallback for pure translation
        }
        
        Rotation2d pathRotation = new Rotation2d(pathAngle);
        Translation2d globalError = new Translation2d(targetX - pose.getX(), targetY - pose.getY());
        
        // Rotate error into the path's frame of reference
        Translation2d localError = globalError.rotateBy(pathRotation.unaryMinus());
        
        // Calculate independent PID efforts
        double pidAlong = alongTrackController.calculate(0.0, localError.getX());
        double pidCross = crossTrackController.calculate(0.0, localError.getY());
        
        // Rotate the resulting effort back into the global field frame
        Translation2d globalPidEffort = new Translation2d(pidAlong, pidCross).rotateBy(pathRotation);
        
        double pidVx = globalPidEffort.getX();
        double pidVy = globalPidEffort.getY();
        double pidOmega = headingController.calculate(pose.getRotation().getRadians(), targetHeading);

        double ffScale = Math.max(0.2, 1.0 - (distanceError / 0.5));
        Translation2d apfVector = calculateAPFVortex(pose, ffVx, ffVy);

        // ESCAPE KICK
        double kickVx = 0.0, kickVy = 0.0;
        if (timeStuck > KICK_TRIGGER_TIME) {
            double kickAngle = pose.getRotation().getRadians() + (Math.PI / 2.0); 
            kickVx = Math.cos(kickAngle) * 2.0; 
            kickVy = Math.sin(kickAngle) * 2.0;
        }

        // SUM & DESATURATE
        double finalVx = (ffVx * ffScale) + pidVx + apfVector.getX() + kickVx;
        double finalVy = (ffVy * ffScale) + pidVy + apfVector.getY() + kickVy;
        double finalOmega = ffOmega + pidOmega; 

        double requestedSpeed = Math.hypot(finalVx, finalVy);
        double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        if (requestedSpeed > maxSpeed) {
            finalVx *= (maxSpeed / requestedSpeed);
            finalVy *= (maxSpeed / requestedSpeed);
        }
        finalVx = applyStaticFrictionCompenation(finalVx);
        finalVy = applyStaticFrictionCompenation(finalVy);

        drive(new ChassisSpeeds(finalVx, finalVy, finalOmega));

        activeTrajectory.sampleAt(closestPathTime, false).ifPresent(
            sample -> field2d.getObject("Ghost").setPose(sample.getPose())
        );

        return false; 
    }

    // =========================================================================
    // API Extensions for POI Tracking
    // =========================================================================
    /** Set a physical coordinate on the field for the robot to look at while driving. */
    public void setPointOfInterest(Translation2d poi) {
        this.pointOfInterest = poi;
    }

    /** Clear the POI to return heading control back to the trajectory. */
    public void clearPointOfInterest() {
        this.pointOfInterest = null;
    }

    // =========================================================================
    // Helper Methods
    // =========================================================================
    private void generateGlobalTrajectory(Pose2d goalPose) {
        isGenerating = true;
        Pose2d startPose = getPose();
        ChassisSpeeds currentRobotSpeeds = driveIO.getDrive().getState().Speeds;
        ChassisSpeeds currentFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentRobotSpeeds, startPose.getRotation());
        ChassisSpeeds currentFieldAcceleration = getCurrentFieldAcceleration(currentRobotSpeeds);

        Trajectory<SwerveSample> prevTraj = (activeTrajectory != null && isFollowingPath) ? activeTrajectory : null;

        CompletableFuture.supplyAsync(() -> {
            List<TrajoPlotPro.TrajectoryPoint> trajectory = solver.solve(
                startPose, currentFieldSpeeds, currentFieldAcceleration, goalPose, 
                waypoints, staticObstacles, rectObstacles, 0.45, 0.05, prevTraj 
            );
            
            if (trajectory == null || trajectory.isEmpty()) return null;
            
            Pose2d[] poses = solver.reconstructToPose2d(trajectory);
            Trajectory<SwerveSample> choreoTraj = solver.convertToChoreo(trajectory);
            return new Object[]{poses, choreoTraj}; 

        }).thenAccept((result) -> {
            if (result != null) {
                field2d.getObject("Robot Trajectory").setPoses((Pose2d[]) result[0]);
                @SuppressWarnings("unchecked")
                Trajectory<SwerveSample> choreoTraj = (Trajectory<SwerveSample>) result[1];
                pendingTrajectory.set(choreoTraj);
            } else {
                System.err.println("Trajectory generation failed!");
            }
            isGenerating = false;
        }).exceptionally(ex -> {
            System.err.println("CRASH IN BACKGROUND PATH GENERATION: " + ex.getMessage());
            ex.printStackTrace();
            isGenerating = false;
            return null;
        });
    }

    private double applyStaticFrictionCompenation(double velocity) {
        if (Math.abs(velocity) == 0.0) {
            return 0.0; 
        } else {
            return velocity + Math.copySign(0.02, velocity); 
        }
    }

    private Translation2d calculateAPFVortex(Pose2d pose, double ffVx, double ffVy) {
        // Kept empty structurally per previous context. Put APF logic back if desired!
        return new Translation2d(0, 0); 
    }

    private boolean checkTrajectoryComplete(double closestPathTime) {
        Pose2d finalPose = activeTrajectory.getFinalPose(false).get();
        return Math.abs(getPose().getMeasureX().in(Meters) - finalPose.getX()) < 0.1 && 
               Math.abs(getPose().getMeasureY().in(Meters) - finalPose.getY()) < 0.1 && 
               Math.abs(getPose().getRotation().relativeTo(finalPose.getRotation()).getRadians()) < Math.toRadians(5.0);
    }

    private void setPath(Trajectory<SwerveSample> trajectory) {
        activeTrajectory = trajectory;
        spatialSampler = new SpatialTrajectorySampler(trajectory, getPose()); 
        isFollowingPath = true;
        isInitialAlignmentReady = false;
        stuckStartTime = -1.0;
    }

    public void setPathfindingTarget(Pose2d targetPose) {
        DriveStates.PATHFINDING.setPose(targetPose);
        setState(DriveStates.PATHFINDING);
    }

    // =========================================================================
    // API & Standard Drive Methods
    // =========================================================================
    public void addObstacle(double x, double y) { staticObstacles.add(new Translation2d(x, y)); }
    public void clearObstacles() { staticObstacles.clear(); }
    public void putTrajectory(String key, Pose2d[] trajectory) { field2d.getObject(key).setPoses(trajectory); }
    
    protected void drive(ChassisSpeeds speeds) {
        driveIO.setControl(new SwerveRequest.FieldCentric().withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }
    protected void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        driveIO.setControl(new SwerveRequest.RobotCentric().withDesaturateWheelSpeeds(true).withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    protected Pose2d getPose() { return driveIO.getDrive().getState().Pose; }
    protected double getVelocityX() { return driveIO.getDrive().getState().Speeds.vxMetersPerSecond; }
    protected double getVelocityY() { return driveIO.getDrive().getState().Speeds.vyMetersPerSecond; }
    protected double getAngularVelocity() { return driveIO.getDrive().getState().Speeds.omegaRadiansPerSecond; }

    private void updateTelemetry() {
        field2d.setRobotPose(getPose());
        Logger.recordOutput("Field", getPose());
        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putString("Drive State", getState().toString());
        SmartDashboard.putNumber("Drive State Time", getStateTime());
    }

    private ChassisSpeeds getCurrentFieldAcceleration(ChassisSpeeds currentRobotSpeeds) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastAccelTimestamp;
        
        ChassisSpeeds rawAccelRobot = new ChassisSpeeds();
        if (dt > 1e-3 && dt < 0.1) { 
            rawAccelRobot = new ChassisSpeeds(
                (currentRobotSpeeds.vxMetersPerSecond - lastRobotSpeeds.vxMetersPerSecond) / dt,
                (currentRobotSpeeds.vyMetersPerSecond - lastRobotSpeeds.vyMetersPerSecond) / dt,
                (currentRobotSpeeds.omegaRadiansPerSecond - lastRobotSpeeds.omegaRadiansPerSecond) / dt
            );
        }

        double alpha = 0.2; 
        double maxA = config.calculateMaxAcceleration();
        
        double clampedVx = MathUtil.clamp(rawAccelRobot.vxMetersPerSecond, -maxA, maxA);
        double clampedVy = MathUtil.clamp(rawAccelRobot.vyMetersPerSecond, -maxA, maxA);
        
        filteredAcceleration.vxMetersPerSecond = (alpha * clampedVx) + ((1 - alpha) * filteredAcceleration.vxMetersPerSecond);
        filteredAcceleration.vyMetersPerSecond = (alpha * clampedVy) + ((1 - alpha) * filteredAcceleration.vyMetersPerSecond);
        filteredAcceleration.omegaRadiansPerSecond = (alpha * rawAccelRobot.omegaRadiansPerSecond) + ((1 - alpha) * filteredAcceleration.omegaRadiansPerSecond);

        lastRobotSpeeds = currentRobotSpeeds;
        lastAccelTimestamp = now;

        return ChassisSpeeds.fromRobotRelativeSpeeds(filteredAcceleration, getPose().getRotation());
    }
}