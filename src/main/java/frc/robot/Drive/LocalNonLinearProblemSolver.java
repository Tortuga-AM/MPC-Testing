package frc.robot.Drive;

import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import static org.wpilib.math.optimization.Constraints.*;
import org.wpilib.math.optimization.Problem;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;

public class LocalNonLinearProblemSolver {
    private static final int N = 50;
    
    // MK5i / Kraken X60 Physical Limits
    private static final double MAX_VELOCITY = 4.39; 
    private static final double MAX_ACCEL = 11.0;    
    private static final double MAX_ANGULAR_VEL = 6.28; // 1 rotation/s
    private static final double MAX_ANGULAR_ACCEL = 12.56;

    public List<TrajectoryPoint> solve(Pose2d startPose, ChassisSpeeds currentSpeeds, ChassisSpeeds currentAcceleration, Pose2d goalPose, List<Pose2d> waypoints) {
        var problem = new Problem();

        // --- 1. HEADING NORMALIZATION ---
        // Prevents the "multiple spins" bug by ensuring angles take the shortest path
        List<Pose2d> fullPath = new ArrayList<>(waypoints);
        fullPath.add(goalPose);
        
        List<Pose2d> optimizedPath = new ArrayList<>();
        double lastTheta = startPose.getRotation().getRadians();
        for (Pose2d wp : fullPath) {
            double delta = MathUtil.angleModulus(wp.getRotation().getRadians() - lastTheta);
            double newTheta = lastTheta + delta;
            optimizedPath.add(new Pose2d(wp.getTranslation(), new Rotation2d(newTheta)));
            lastTheta = newTheta;
        }

        // --- 2. DISTANCE-BASED INDEXING ---
        // Prevents "corner cutting" by spacing waypoints correctly in time
        double[] dists = new double[optimizedPath.size()];
        double totalDist = startPose.getTranslation().getDistance(optimizedPath.get(0).getTranslation());
        dists[0] = totalDist;
        for (int i = 1; i < optimizedPath.size(); i++) {
            totalDist += optimizedPath.get(i-1).getTranslation().getDistance(optimizedPath.get(i).getTranslation());
            dists[i] = totalDist;
        }

        // Decision Variables
        Variable T = problem.decisionVariable();
        VariableMatrix X = problem.decisionVariable(6, N + 1);
        VariableMatrix U = problem.decisionVariable(3, N);

        // --- 3. SMOOTHED OBJECTIVE ---
        // Initialize with a decision variable starting at 0.0
        Variable accelPenalty = problem.decisionVariable();
        problem.subjectTo(eq(accelPenalty, 0.0)); 

        for (int k = 0; k < N; k++) {
            // Adding the squared accelerations to the penalty
            // U.get(0, k) is ax, U.get(1, k) is ay, U.get(2, k) is alpha
            accelPenalty = accelPenalty.plus(Variable.pow(U.get(0, k),2))
                                    .plus(Variable.pow(U.get(1, k),2))
                                    .plus(Variable.pow(U.get(2, k),2).times(0.5));
        }
        problem.minimize(T.plus(accelPenalty.times(0.0000))); 
        problem.subjectTo(ge(T, 0.5)); 

        Variable dt = T.div(N);

        for (int k = 0; k < N; k++) {
            Variable vx_k = X.get(3, k);
            Variable vy_k = X.get(4, k);
            Variable omega_k = X.get(5, k);
            Variable ax_k = U.get(0, k);
            Variable ay_k = U.get(1, k);
            Variable alpha_k = U.get(2, k);

            // Kinematics (Second-Order)
            problem.subjectTo(eq(X.get(0, k + 1), X.get(0, k).plus(vx_k.times(dt)).plus(ax_k.times(dt.times(dt).times(0.5)))));
            problem.subjectTo(eq(X.get(1, k + 1), X.get(1, k).plus(vy_k.times(dt)).plus(ay_k.times(dt.times(dt).times(0.5)))));
            problem.subjectTo(eq(X.get(2, k + 1), X.get(2, k).plus(omega_k.times(dt)).plus(alpha_k.times(dt.times(dt).times(0.5)))));

            // Velocity Integration
            problem.subjectTo(eq(X.get(3, k + 1), vx_k.plus(ax_k.times(dt))));
            problem.subjectTo(eq(X.get(4, k + 1), vy_k.plus(ay_k.times(dt))));
            problem.subjectTo(eq(X.get(5, k + 1), omega_k.plus(alpha_k.times(dt))));

            // Limits
            problem.subjectTo(le(vx_k.times(vx_k).plus(vy_k.times(vy_k)), Math.pow(MAX_VELOCITY, 2)));
            problem.subjectTo(le(ax_k.times(ax_k).plus(ay_k.times(ay_k)), Math.pow(MAX_ACCEL, 2)));
            problem.subjectTo(le(Variable.abs(omega_k), MAX_ANGULAR_VEL));
            problem.subjectTo(le(Variable.abs(alpha_k), MAX_ANGULAR_ACCEL));
        }

        // Start Conditions
        problem.subjectTo(eq(X.get(0, 0), startPose.getX()));
        problem.subjectTo(eq(X.get(1, 0), startPose.getY()));
        problem.subjectTo(eq(X.get(2, 0), startPose.getRotation().getRadians()));
        problem.subjectTo(eq(X.get(3, 0), currentSpeeds.vxMetersPerSecond));
        problem.subjectTo(eq(X.get(4, 0), currentSpeeds.vyMetersPerSecond));
        problem.subjectTo(eq(X.get(5, 0), currentSpeeds.omegaRadiansPerSecond));
        problem.subjectTo(eq(U.get(0, 0), currentAcceleration.vxMetersPerSecond));
        problem.subjectTo(eq(U.get(1, 0), currentAcceleration.vyMetersPerSecond));
        problem.subjectTo(eq(U.get(2, 0), currentAcceleration.omegaRadiansPerSecond));

        // Waypoints & Goal
        for (int i = 0; i < optimizedPath.size(); i++) {
            Pose2d wp = optimizedPath.get(i);
            int k = (int) Math.round((dists[i] / totalDist) * N);
            k = MathUtil.clamp(k, 1, N);
            
            problem.subjectTo(eq(X.get(0, k), wp.getX()));
            problem.subjectTo(eq(X.get(1, k), wp.getY()));
            problem.subjectTo(eq(X.get(2, k), wp.getRotation().getRadians()));

            if (i == optimizedPath.size() - 1) {
                problem.subjectTo(eq(X.get(3, k), 0.0));
                problem.subjectTo(eq(X.get(4, k), 0.0));
                problem.subjectTo(eq(X.get(5, k), 0.0));
            }
        }

        T.setValue(3.0); 
        System.out.println("Solve Status: " + problem.solve());
        System.out.println("Sleipnir Solve Complete. Total Time: " + T.value() + " seconds");

        List<TrajectoryPoint> trajectory = getSolution(T, X, U);
        problem.close();
        return trajectory;
    }

    public record TrajectoryPoint(
        double time, double x, double y, double theta, 
        double vx, double vy, double omega,
        double ax, double ay, double alpha,
        double[] moduleForcesX, double[] moduleForcesY
    ) {}

    public List<TrajectoryPoint> getSolution(Variable T, VariableMatrix X, VariableMatrix U) {
        List<TrajectoryPoint> trajectory = new ArrayList<>();
        double totalTime = T.value();
        double dtValue = totalTime / N;

        final double MASS_KG = 60.0; 
        final double MOI_KGM2 = 6.0; 
        final double TRACK_WIDTH_HALF = 0.45; 

        for (int k = 0; k <= N; k++) {
            double ax = (k < N) ? U.get(0, k).value() : 0.0;
            double ay = (k < N) ? U.get(1, k).value() : 0.0;
            double alpha = (k < N) ? U.get(2, k).value() : 0.0;

            double fx = (ax * MASS_KG) / 4.0;
            double fy = (ay * MASS_KG) / 4.0;
            double torqueForce = (alpha * MOI_KGM2) / (4.0 * TRACK_WIDTH_HALF);

            double[] mFX = { fx - torqueForce, fx + torqueForce, fx - torqueForce, fx + torqueForce };
            double[] mFY = { fy + torqueForce, fy + torqueForce, fy - torqueForce, fy - torqueForce };

            trajectory.add(new TrajectoryPoint(
                k * dtValue,
                X.get(0, k).value(), X.get(1, k).value(), X.get(2, k).value(),
                X.get(3, k).value(), X.get(4, k).value(), X.get(5, k).value(),
                ax, ay, alpha, mFX, mFY
            ));
        }
        return trajectory;
    }

    public Pose2d[] reconstructToPose2d(List<TrajectoryPoint> trajectory) {
        List<Pose2d> poses = new ArrayList<>();
        for (TrajectoryPoint point : trajectory) {
            poses.add(new Pose2d(point.x, point.y, new Rotation2d(point.theta)));
        }
        return poses.toArray(new Pose2d[0]);
    }

    public Trajectory<SwerveSample> convertToChoreo(List<TrajectoryPoint> sleipnirPath) {
        List<SwerveSample> samples = new ArrayList<>();
        for (TrajectoryPoint pt : sleipnirPath) {
            samples.add(new SwerveSample(
                pt.time(), pt.x(), pt.y(), pt.theta(),
                pt.vx(), pt.vy(), pt.omega(),
                pt.ax(), pt.ay(), pt.alpha(),
                pt.moduleForcesX(), pt.moduleForcesY()
            ));
        }
        return new Trajectory<>("SleipnirPath", samples, List.of(), List.of());
    }
}