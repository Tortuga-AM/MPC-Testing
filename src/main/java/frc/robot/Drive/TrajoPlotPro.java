package frc.robot.Drive;

import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import static org.wpilib.math.optimization.Constraints.*;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;

public class TrajoPlotPro {
    private static final int N = 75; 

    public record RobotConfig(
        double massLbs,
        double moi,
        double wheelRadiusInches,
        double wheelCOF,
        double gearRatio,
        double maxMotorRPM,
        double maxMotorTorqueNm,
        Translation2d[] moduleOffsets,
        double maxVelocityOverride, 
        double maxAccelOverride    
    ) {
        public double massKg() { return massLbs * 0.453592; }
        public double wheelRadiusMeters() { return wheelRadiusInches * 0.0254; }
        
        public double calculateMaxVelocity() {
            double theoretical = (maxMotorRPM / 60.0 / gearRatio) * 2 * Math.PI * wheelRadiusMeters();
            return Math.min(theoretical, maxVelocityOverride);
        }

        public double calculateMaxAcceleration() {
            double motorLimitedAccel = (maxMotorTorqueNm * gearRatio / wheelRadiusMeters() * 4.0) / massKg();
            double tractionLimitedAccel = 9.806 * wheelCOF;
            return Math.min(Math.min(motorLimitedAccel, tractionLimitedAccel), maxAccelOverride);
        }
    }

    private final RobotConfig config;

    public TrajoPlotPro(RobotConfig config) {
        this.config = config;
    }

    public List<TrajectoryPoint> solve(Pose2d startPose, ChassisSpeeds currentSpeeds, ChassisSpeeds currentAccel, Pose2d goalPose, List<Pose2d> waypoints) {
        var problem = new Problem();

        // 1. Path Processing & Heading Normalization
        List<Pose2d> fullPath = new ArrayList<>(waypoints);
        fullPath.add(goalPose);
        List<Pose2d> optimizedPath = normalizeHeadings(startPose, fullPath);

        // 2. Decision Variables
        Variable T = problem.decisionVariable();
        VariableMatrix X = problem.decisionVariable(6, N + 1); 
        VariableMatrix U = problem.decisionVariable(3, N);     

        // 3. Hardware Constants
        double maxV = config.calculateMaxVelocity();
        double maxA = config.calculateMaxAcceleration();

        // 4. STABILITY: Smart Velocity-Aware Initial Guess
        applySmartGuess(X, T, startPose, currentSpeeds, optimizedPath);

        // 5. Objective: Time + Smoothness + Control Effort
        Variable cost = problem.decisionVariable();
        problem.subjectTo(eq(cost, 0.0));
        for (int k = 0; k < N; k++) {
            // Penalizing acceleration (U^2) prevents the "random teleport" behavior
            cost = cost.plus(Variable.pow(U.get(0, k), 2))
                       .plus(Variable.pow(U.get(1, k), 2))
                       .plus(Variable.pow(U.get(2, k), 2).times(0.5));
        }
        // Small weight on cost (0.0001) keeps T as the primary priority
        problem.minimize(T.plus(cost.times(0.0001)));
        problem.subjectTo(ge(T, 0.1));

        Variable dt = T.div(N);

        // 6. Constraints Loop
        for (int k = 0; k < N; k++) {
            Variable vx_k = X.get(3, k);
            Variable vy_k = X.get(4, k);
            Variable omega_k = X.get(5, k);
            Variable ax_k = U.get(0, k);
            Variable ay_k = U.get(1, k);
            Variable alpha_k = U.get(2, k);

            // Kinematics: Stability Weighted 2nd Order Integration
            problem.subjectTo(eq(X.get(0, k+1), X.get(0, k).plus(vx_k.times(dt)).plus(ax_k.times(Variable.pow(dt, 2).times(0.5)))));
            problem.subjectTo(eq(X.get(1, k+1), X.get(1, k).plus(vy_k.times(dt)).plus(ay_k.times(Variable.pow(dt, 2).times(0.5)))));
            problem.subjectTo(eq(X.get(2, k+1), X.get(2, k).plus(omega_k.times(dt)).plus(alpha_k.times(Variable.pow(dt, 2).times(0.5)))));
            
            problem.subjectTo(eq(X.get(3, k+1), vx_k.plus(ax_k.times(dt))));
            problem.subjectTo(eq(X.get(4, k+1), vy_k.plus(ay_k.times(dt))));
            problem.subjectTo(eq(X.get(5, k+1), omega_k.plus(alpha_k.times(dt))));

            // Module-Level Velocity Limits
            for (Translation2d offset : config.moduleOffsets()) {
                Variable v_wheel_x = vx_k.minus(omega_k.times(offset.getY()));
                Variable v_wheel_y = vy_k.plus(omega_k.times(offset.getX()));
                problem.subjectTo(le(Variable.pow(v_wheel_x, 2).plus(Variable.pow(v_wheel_y, 2)), Math.pow(maxV, 2)));
            }

            // Chassis Acceleration Limits (Hard Limit)
            problem.subjectTo(le(Variable.pow(ax_k, 2).plus(Variable.pow(ay_k, 2)), Math.pow(maxA, 2)));
        }

        // 7. Boundary Conditions
        pinState(problem, X, U, 0, startPose, currentSpeeds, currentAccel);
        
        double totalDist = calculateTotalDist(startPose, optimizedPath);
        double distAcc = 0;
        for (int i = 0; i < optimizedPath.size(); i++) {
            Pose2d prev = (i == 0) ? startPose : optimizedPath.get(i-1);
            distAcc += prev.getTranslation().getDistance(optimizedPath.get(i).getTranslation());
            
            int k = (int) Math.round((distAcc / totalDist) * N);
            k = MathUtil.clamp(k, 1, N);
            
            problem.subjectTo(eq(X.get(0, k), optimizedPath.get(i).getX()));
            problem.subjectTo(eq(X.get(1, k), optimizedPath.get(i).getY()));
            problem.subjectTo(eq(X.get(2, k), optimizedPath.get(i).getRotation().getRadians()));
            
            if (i == optimizedPath.size() - 1) { 
                problem.subjectTo(eq(X.get(3, N), 0));
                problem.subjectTo(eq(X.get(4, N), 0));
                problem.subjectTo(eq(X.get(5, N), 0));
            }
        }

        var status = problem.solve();
        
        // Critical: Only return the path if the solver succeeded
        System.out.println("Sleipnir Solve Complete. Total Time: " + T.value() + " seconds");
        System.out.println("Final Cost: " + cost.value());
        if (status != ExitStatus.SUCCESS) {
            System.err.println("SOLVER FAILED: " + status);
            return List.of(); 
        }
        List<TrajectoryPoint> solution = getSolution(T, X, U);
        problem.close();
        return solution;
    }

    private void applySmartGuess(VariableMatrix X, Variable T, Pose2d start, ChassisSpeeds current, List<Pose2d> path) {
        // Estimate time based on 70% of max velocity to be realistic
        double estT = calculateTotalDist(start, path) / (config.calculateMaxVelocity() * 0.7);
        T.setValue(Math.max(estT, 1.5));

        for (int k = 0; k <= N; k++) {
            double frac = (double) k / N;
            Pose2d goal = path.get(path.size() - 1);
            
            // Linear Interpolation for Pose
            X.get(0, k).setValue(start.getX() + (goal.getX() - start.getX()) * frac);
            X.get(1, k).setValue(start.getY() + (goal.getY() - start.getY()) * frac);
            X.get(2, k).setValue(start.getRotation().getRadians() + 
                                (goal.getRotation().getRadians() - start.getRotation().getRadians()) * frac);
            
            // Seed Velocity: Start with current velocity, fade to zero at end
            X.get(3, k).setValue(current.vxMetersPerSecond * (1.0 - frac));
            X.get(4, k).setValue(current.vyMetersPerSecond * (1.0 - frac));
            X.get(5, k).setValue(current.omegaRadiansPerSecond * (1.0 - frac));
        }
    }

    private List<Pose2d> normalizeHeadings(Pose2d start, List<Pose2d> path) {
        List<Pose2d> optimized = new ArrayList<>();
        double lastTheta = start.getRotation().getRadians();
        for (Pose2d wp : path) {
            double delta = MathUtil.angleModulus(wp.getRotation().getRadians() - lastTheta);
            double nextTheta = lastTheta + delta;
            optimized.add(new Pose2d(wp.getTranslation(), Rotation2d.fromRadians(nextTheta)));
            lastTheta = nextTheta;
        }
        return optimized;
    }

    private void pinState(Problem p, VariableMatrix X, VariableMatrix U, int k, Pose2d pose, ChassisSpeeds s, ChassisSpeeds a) {
        p.subjectTo(eq(X.get(0, k), pose.getX()));
        p.subjectTo(eq(X.get(1, k), pose.getY()));
        p.subjectTo(eq(X.get(2, k), pose.getRotation().getRadians()));
        p.subjectTo(eq(X.get(3, k), s.vxMetersPerSecond));
        p.subjectTo(eq(X.get(4, k), s.vyMetersPerSecond));
        p.subjectTo(eq(X.get(5, k), s.omegaRadiansPerSecond));
        if (k < N) {
            p.subjectTo(eq(U.get(0, k), a.vxMetersPerSecond));
            p.subjectTo(eq(U.get(1, k), a.vyMetersPerSecond));
            p.subjectTo(eq(U.get(2, k), a.omegaRadiansPerSecond));
        }
    }

    private double calculateTotalDist(Pose2d start, List<Pose2d> path) {
        double d = start.getTranslation().getDistance(path.get(0).getTranslation());
        for (int i = 1; i < path.size(); i++) d += path.get(i-1).getTranslation().getDistance(path.get(i).getTranslation());
        return Math.max(d, 0.01);
    }

    public record TrajectoryPoint(double time, double x, double y, double theta, double vx, double vy, double omega, double ax, double ay, double alpha, double[] moduleForcesX, double[] moduleForcesY) {}

    private List<TrajectoryPoint> getSolution(Variable T, VariableMatrix X, VariableMatrix U) {
        List<TrajectoryPoint> trajectory = new ArrayList<>();
        double dt = T.value() / N;
        double mass = config.massKg();
        double moi = config.moi();

        double avgRadius = 0;
        for(var offset : config.moduleOffsets()) avgRadius += offset.getNorm();
        avgRadius /= 4.0;

        for (int k = 0; k <= N; k++) {
            double ax = (k < N) ? U.get(0, k).value() : 0;
            double ay = (k < N) ? U.get(1, k).value() : 0;
            double alpha = (k < N) ? U.get(2, k).value() : 0;

            double fx = (ax * mass) / 4.0;
            double fy = (ay * mass) / 4.0;
            double torqueF = (alpha * moi) / (4.0 * avgRadius);

            trajectory.add(new TrajectoryPoint(
                k * dt, X.get(0, k).value(), X.get(1, k).value(), X.get(2, k).value(),
                X.get(3, k).value(), X.get(4, k).value(), X.get(5, k).value(),
                ax, ay, alpha, 
                new double[]{fx - torqueF, fx + torqueF, fx - torqueF, fx + torqueF}, 
                new double[]{fy + torqueF, fy + torqueF, fy - torqueF, fy - torqueF}
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

    public Trajectory<SwerveSample> convertToChoreo(List<TrajectoryPoint> path) {
        if (path.isEmpty()) return null;
        List<SwerveSample> samples = new ArrayList<>();
        for (TrajectoryPoint p : path) {
            samples.add(new SwerveSample(p.time, p.x, p.y, p.theta, p.vx, p.vy, p.omega, p.ax, p.ay, p.alpha, p.moduleForcesX, p.moduleForcesY));
        }
        return new Trajectory<>("ProPath", samples, List.of(), List.of());
    }
}