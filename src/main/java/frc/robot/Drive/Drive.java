package frc.robot.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Drive.DriveConstants.ANGULAR_VELOCITY_LIMIT;
import static frc.robot.GlobalConstants.Controllers.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;

public class Drive extends Subsystem<DriveStates> {
    private static Drive instance;

    // --- Hardware & Config ---
    private final DriveIO driveIO;
    private final Field2d field2d = new Field2d();
    
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
    }

    // =========================================================================
    // Main State Machine
    // =========================================================================
    @Override
    public void runState() {
        updateTelemetry();
        
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
            default -> driveIO.setControl(new SwerveRequest.SwerveDriveBrake());
        }
    }

    private void updateTelemetry() {
        Pose2d pose = getPose();
        field2d.setRobotPose(pose);
        Logger.recordOutput("Drive/Pose", pose);
        SmartDashboard.putData("Field", field2d);
    }

    private void drive(ChassisSpeeds speeds) {
        driveIO.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    public Pose2d getPose() {
        return driveIO.getDrive().getState().Pose;
    }
}