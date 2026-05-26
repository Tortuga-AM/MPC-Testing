package frc.robot.Drive.AutoAlign;

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

    public record RobotConfig(
        double massLbs, double moi, double wheelRadiusInches, double wheelCOF,
        double gearRatio, double maxMotorRPM, double maxMotorTorqueNm,
        Translation2d[] moduleOffsets, double maxVelocityOverride, double maxAccelOverride    
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

        public double getDriveBaseRadius() {
            double radius = 0;
            for(var offset : moduleOffsets) radius += offset.getNorm();
            return radius / moduleOffsets.length;
        }
    }

    public record RectObstacle(double x, double y, double width, double height) {}

    private final RobotConfig config;
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;

    public TrajoPlotPro(RobotConfig config) {
        this.config = config;
    }

    public List<TrajectoryPoint> solve(Pose2d rawStartPose, ChassisSpeeds rawCurrentSpeeds, ChassisSpeeds rawCurrentAccel, Pose2d rawGoalPose, List<Pose2d> rawWaypoints, 
                                       List<Translation2d> circularObstacles, List<RectObstacle> rectObstacles, 
                                       double robotBumperRadiusMeters, double additionalSafeBufferMeters, Trajectory<SwerveSample> previousTrajectory) {
        
        double maxV = config.calculateMaxVelocity();
        double maxA = config.calculateMaxAcceleration();
        double avoidanceRadius = robotBumperRadiusMeters + additionalSafeBufferMeters;
        double maxOmega = maxV / config.getDriveBaseRadius();

        Pose2d boundsClampedStart = clampPoseToField(rawStartPose, robotBumperRadiusMeters);
        Pose2d goalPose = clampPoseToField(rawGoalPose, robotBumperRadiusMeters);
        Pose2d startPose = pushOutOfObstacles(boundsClampedStart, circularObstacles, rectObstacles, avoidanceRadius);

        List<Pose2d> waypoints = new ArrayList<>();
        if (rawWaypoints != null) {
            for (Pose2d wp : rawWaypoints) waypoints.add(clampPoseToField(wp, robotBumperRadiusMeters));
        }

        double currentV = Math.hypot(rawCurrentSpeeds.vxMetersPerSecond, rawCurrentSpeeds.vyMetersPerSecond);
        ChassisSpeeds safeSpeeds = rawCurrentSpeeds;
        if (currentV > maxV) {
            safeSpeeds = new ChassisSpeeds(
                rawCurrentSpeeds.vxMetersPerSecond * (maxV / currentV),
                rawCurrentSpeeds.vyMetersPerSecond * (maxV / currentV),
                MathUtil.clamp(rawCurrentSpeeds.omegaRadiansPerSecond, -maxOmega, maxOmega)
            );
        }

        double currentA = Math.hypot(rawCurrentAccel.vxMetersPerSecond, rawCurrentAccel.vyMetersPerSecond);
        ChassisSpeeds safeAccel = rawCurrentAccel;
        if (currentA > maxA) {
            safeAccel = new ChassisSpeeds(
                rawCurrentAccel.vxMetersPerSecond * (maxA / currentA),
                rawCurrentAccel.vyMetersPerSecond * (maxA / currentA),
                rawCurrentAccel.omegaRadiansPerSecond 
            );
        }

        double totalDist = startPose.getTranslation().getDistance(goalPose.getTranslation());
        if (!waypoints.isEmpty()) {
            totalDist = startPose.getTranslation().getDistance(waypoints.get(0).getTranslation());
            for (int i = 1; i < waypoints.size(); i++) {
                totalDist += waypoints.get(i-1).getTranslation().getDistance(waypoints.get(i).getTranslation());
            }
            totalDist += waypoints.get(waypoints.size()-1).getTranslation().getDistance(goalPose.getTranslation());
        }

        int numWaypoints = waypoints.size();
        int numCircs = (circularObstacles == null) ? 0 : circularObstacles.size();
        int numRects = (rectObstacles == null) ? 0 : rectObstacles.size();
        
        int calculatedN = 10 + (int)(totalDist * 4.0) + (numWaypoints * 3) + ((numCircs + numRects) * 2);
        final int N = MathUtil.clamp(calculatedN, 15, 45);

        var problem = new Problem();
        Variable T = problem.decisionVariable();
        VariableMatrix X = problem.decisionVariable(6, N + 1); 
        VariableMatrix U = problem.decisionVariable(3, N);     

        VariableMatrix slackObs = problem.decisionVariable(1, N + 1);
        VariableMatrix slackWp = problem.decisionVariable(1, N + 1);

        applySmartGuess(X, T, startPose, safeSpeeds, goalPose, previousTrajectory, N, circularObstacles, rectObstacles, avoidanceRadius);

        Variable cost = problem.decisionVariable();
        problem.subjectTo(eq(cost, 0.0));
        
        for (int k = 1; k < N; k++) {
            Variable jerkX = U.get(0, k).minus(U.get(0, k-1));
            Variable jerkY = U.get(1, k).minus(U.get(1, k-1));
            Variable jerkOmega = U.get(2, k).minus(U.get(2, k-1));
            
            Variable vx = X.get(3, k);
            Variable vy = X.get(4, k);
            Variable omega = X.get(5, k);

            Variable speedSq = Variable.pow(vx, 2).plus(Variable.pow(vy, 2));
            Variable centripetalTax = speedSq.times(Variable.pow(omega, 2));
            
            cost = cost.plus(Variable.pow(jerkX, 2))
                       .plus(Variable.pow(jerkY, 2))
                       .plus(Variable.pow(jerkOmega, 2).times(0.5))
                       .plus(centripetalTax.times(0.005));
        }

        for (int k = 0; k <= N; k++) {
            problem.subjectTo(ge(slackObs.get(0, k), 0.0)); 
            problem.subjectTo(ge(slackWp.get(0, k), 0.0));
            cost = cost.plus(slackObs.get(0, k).times(1000.0))
                       .plus(slackWp.get(0, k).times(500.0));
        }
        
        problem.minimize(T.plus(cost.times(0.005)));
        problem.subjectTo(ge(T, 0.1));
        problem.subjectTo(le(T, 15.0)); 

        Variable dt = T.div(N);

        for (int k = 0; k < N; k++) {
            Variable x_pos = X.get(0, k);
            Variable y_pos = X.get(1, k);
            Variable vx_k = X.get(3, k);
            Variable vy_k = X.get(4, k);
            Variable omega_k = X.get(5, k);
            Variable ax_k = U.get(0, k);
            Variable ay_k = U.get(1, k);
            Variable alpha_k = U.get(2, k);

            problem.subjectTo(eq(X.get(0, k+1), x_pos.plus(vx_k.times(dt)).plus(ax_k.times(Variable.pow(dt, 2).times(0.5)))));
            problem.subjectTo(eq(X.get(1, k+1), y_pos.plus(vy_k.times(dt)).plus(ay_k.times(Variable.pow(dt, 2).times(0.5)))));
            problem.subjectTo(eq(X.get(2, k+1), X.get(2, k).plus(omega_k.times(dt)).plus(alpha_k.times(Variable.pow(dt, 2).times(0.5)))));
            problem.subjectTo(eq(X.get(3, k+1), vx_k.plus(ax_k.times(dt))));
            problem.subjectTo(eq(X.get(4, k+1), vy_k.plus(ay_k.times(dt))));
            problem.subjectTo(eq(X.get(5, k+1), omega_k.plus(alpha_k.times(dt))));

            for (Translation2d offset : config.moduleOffsets()) {
                Variable v_wheel_x = vx_k.minus(omega_k.times(offset.getY()));
                Variable v_wheel_y = vy_k.plus(omega_k.times(offset.getX()));
                problem.subjectTo(le(Variable.pow(v_wheel_x, 2).plus(Variable.pow(v_wheel_y, 2)), Math.pow(maxV, 2)));
            }
            problem.subjectTo(le(Variable.pow(ax_k, 2).plus(Variable.pow(ay_k, 2)), Math.pow(maxA, 2)));
        }

        for (int k = 0; k <= N; k++) {
            Variable x_pos = X.get(0, k);
            Variable y_pos = X.get(1, k);
            Variable currentSlack = slackObs.get(0, k);

            problem.subjectTo(ge(x_pos.plus(currentSlack), robotBumperRadiusMeters));
            problem.subjectTo(le(x_pos.minus(currentSlack), FIELD_LENGTH - robotBumperRadiusMeters));
            problem.subjectTo(ge(y_pos.plus(currentSlack), robotBumperRadiusMeters));
            problem.subjectTo(le(y_pos.minus(currentSlack), FIELD_WIDTH - robotBumperRadiusMeters));

            double rSquared = avoidanceRadius * avoidanceRadius;
            if (circularObstacles != null) {
                for (Translation2d obs : circularObstacles) {
                    Variable dx = x_pos.minus(obs.getX());
                    Variable dy = y_pos.minus(obs.getY());
                    problem.subjectTo(ge(Variable.pow(dx, 2).plus(Variable.pow(dy, 2)).plus(currentSlack), rSquared));
                }
            }

            if (rectObstacles != null) {
                for (RectObstacle rect : rectObstacles) {
                    double hW = (rect.width() / 2.0) + avoidanceRadius;
                    double hH = (rect.height() / 2.0) + avoidanceRadius;

                    Variable dx = x_pos.minus(rect.x());
                    Variable dy = y_pos.minus(rect.y());
                    Variable xTerm = Variable.pow(dx.div(hW), 4);
                    Variable yTerm = Variable.pow(dy.div(hH), 4);

                    problem.subjectTo(ge(xTerm.plus(yTerm).plus(currentSlack), 1.0));
                }
            }
        }

        pinState(problem, X, U, 0, startPose, safeSpeeds, safeAccel, N);
        
        problem.subjectTo(eq(X.get(0, N), goalPose.getX()));
        problem.subjectTo(eq(X.get(1, N), goalPose.getY()));
        
        double normalizedFinalHeading = startPose.getRotation().getRadians() + 
            MathUtil.angleModulus(goalPose.getRotation().getRadians() - startPose.getRotation().getRadians());
        problem.subjectTo(eq(X.get(2, N), normalizedFinalHeading));

        problem.subjectTo(eq(X.get(3, N), 0));
        problem.subjectTo(eq(X.get(4, N), 0));
        problem.subjectTo(eq(X.get(5, N), 0));

        if (!waypoints.isEmpty()) {
            List<Pose2d> fullPath = new ArrayList<>();
            fullPath.add(startPose);
            fullPath.addAll(waypoints);
            fullPath.add(goalPose);

            double accDist = 0;
            for (int i = 1; i < fullPath.size(); i++) {
                accDist += fullPath.get(i - 1).getTranslation().getDistance(fullPath.get(i).getTranslation());
            }
            
            double distAcc = 0;
            for (int i = 0; i < waypoints.size(); i++) {
                Pose2d wp = waypoints.get(i);
                Pose2d prev = (i == 0) ? startPose : waypoints.get(i - 1);
                distAcc += prev.getTranslation().getDistance(wp.getTranslation());
                
                int k = (int) Math.round((distAcc / accDist) * N);
                k = MathUtil.clamp(k, 1, N - 1);
                
                // Waypoint Translational Slack Constraint
                double tolSq = 0.25 * 0.25; 
                Variable dx = X.get(0, k).minus(wp.getX());
                Variable dy = X.get(1, k).minus(wp.getY());
                problem.subjectTo(le(Variable.pow(dx, 2).plus(Variable.pow(dy, 2)), slackWp.get(0, k).plus(tolSq)));

                // Waypoint Rotational Slack Constraint
                double wpHeading = startPose.getRotation().getRadians() + 
                    MathUtil.angleModulus(wp.getRotation().getRadians() - startPose.getRotation().getRadians());
                Variable headingError = X.get(2, k).minus(wpHeading);
                
                double headingTolSq = Math.pow(Math.toRadians(20.0), 2); // +/- 20 degrees is "close enough" without slack penalties
                problem.subjectTo(le(Variable.pow(headingError, 2), slackWp.get(0, k).plus(headingTolSq)));
            }
        }

        var status = problem.solve();
        
        if (status != ExitStatus.SUCCESS) {
            System.err.println("SLEIPNIR SOLVER FAILED: " + status);
            return List.of(); 
        }

        System.out.println("Sleipnir Solve Complete. Knots: " + N + " | Time: " + T.value() + " seconds");
        List<TrajectoryPoint> solution = getSolution(T, X, U, N);
        problem.close();
        return solution;
    }

    private Pose2d clampPoseToField(Pose2d pose, double bumperRadius) {
        double clampedX = MathUtil.clamp(pose.getX(), bumperRadius + 0.05, FIELD_LENGTH - bumperRadius - 0.05);
        double clampedY = MathUtil.clamp(pose.getY(), bumperRadius + 0.05, FIELD_WIDTH - bumperRadius - 0.05);
        return new Pose2d(clampedX, clampedY, pose.getRotation());
    }

    private Pose2d pushOutOfObstacles(Pose2d pose, List<Translation2d> circs, List<RectObstacle> rects, double radius) {
        double px = pose.getX();
        double py = pose.getY();

        if (circs != null) {
            for (Translation2d obs : circs) {
                double dx = px - obs.getX();
                double dy = py - obs.getY();
                double dist = Math.hypot(dx, dy);
                if (dist < radius && dist > 0.01) {
                    px = obs.getX() + (dx / dist) * (radius + 0.05);
                    py = obs.getY() + (dy / dist) * (radius + 0.05);
                }
            }
        }

        if (rects != null) {
            for (RectObstacle rect : rects) {
                double hW = (rect.width() / 2.0) + radius;
                double hH = (rect.height() / 2.0) + radius;
                if (px > rect.x() - hW && px < rect.x() + hW && py > rect.y() - hH && py < rect.y() + hH) {
                    double dLeft = Math.abs(px - (rect.x() - hW));
                    double dRight = Math.abs(px - (rect.x() + hW));
                    double dBottom = Math.abs(py - (rect.y() - hH));
                    double dTop = Math.abs(py - (rect.y() + hH));
                    
                    double min = Math.min(Math.min(dLeft, dRight), Math.min(dBottom, dTop));
                    if (min == dLeft) px = rect.x() - hW - 0.05;
                    else if (min == dRight) px = rect.x() + hW + 0.05;
                    else if (min == dBottom) py = rect.y() - hH - 0.05;
                    else py = rect.y() + hH + 0.05;
                }
            }
        }
        return new Pose2d(px, py, pose.getRotation());
    }

    private void applySmartGuess(VariableMatrix X, Variable T, Pose2d start, ChassisSpeeds current, Pose2d goal, Trajectory<SwerveSample> previousTrajectory, int N, List<Translation2d> circularObstacles, List<RectObstacle> rectObstacles, double avoidanceRadius) {
        if (previousTrajectory != null && previousTrajectory.getPoses().length != 0) {
            double totalOldTime = previousTrajectory.getTotalTime();
            T.setValue(totalOldTime);

            for (int k = 0; k <= N; k++) {
                double frac = (double) k / N;
                double queryTime = frac * totalOldTime;
                SwerveSample oldSample = previousTrajectory.sampleAt(queryTime, false).get();
                
                X.get(0, k).setValue(oldSample.x);
                X.get(1, k).setValue(oldSample.y);
                X.get(2, k).setValue(oldSample.heading);
                X.get(3, k).setValue(oldSample.vx);
                X.get(4, k).setValue(oldSample.vy);
                X.get(5, k).setValue(oldSample.omega);
            }
        } else {
            Translation2d currentVelocity = new Translation2d(current.vxMetersPerSecond, current.vyMetersPerSecond);
            List<Translation2d> geomPath = AStarPathfind.findPath(start.getTranslation(), goal.getTranslation(), circularObstacles, rectObstacles, avoidanceRadius);

            double totalGeomDist = 0;
            for (int i = 0; i < geomPath.size() - 1; i++) {
                totalGeomDist += geomPath.get(i).getDistance(geomPath.get(i+1));
            }

            double estT = Math.max((totalGeomDist / (config.calculateMaxVelocity() * 0.7)), 0.5);
            T.setValue(estT);

            double startTheta = start.getRotation().getRadians();
            double goalTheta = startTheta + MathUtil.angleModulus(goal.getRotation().getRadians() - startTheta);

            for (int k = 0; k <= N; k++) {
                double frac = (double) k / N;
                double targetDist = frac * totalGeomDist;
                
                Translation2d point = interpolatePointAtDistance(geomPath, targetDist);

                X.get(0, k).setValue(point.getX());
                X.get(1, k).setValue(point.getY());
                X.get(2, k).setValue(startTheta + (goalTheta - startTheta) * frac);
                X.get(3, k).setValue(current.vxMetersPerSecond * (1.0 - frac));
                X.get(4, k).setValue(current.vyMetersPerSecond * (1.0 - frac));
                X.get(5, k).setValue(current.omegaRadiansPerSecond * (1.0 - frac));
            }
        }
    }

    private Translation2d interpolatePointAtDistance(List<Translation2d> path, double targetDist) {
        if (path.isEmpty()) return new Translation2d();
        if (path.size() == 1) return path.get(0);
        
        double currentDist = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            double segLen = path.get(i).getDistance(path.get(i+1));
            if (currentDist + segLen >= targetDist) {
                double t = (targetDist - currentDist) / segLen;
                return path.get(i).interpolate(path.get(i+1), t);
            }
            currentDist += segLen;
        }
        return path.get(path.size() - 1);
    }

    private void pinState(Problem p, VariableMatrix X, VariableMatrix U, int k, Pose2d pose, ChassisSpeeds s, ChassisSpeeds a, int N) {
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

    public record TrajectoryPoint(double time, double x, double y, double theta, double vx, double vy, double omega, double ax, double ay, double alpha, double[] moduleForcesX, double[] moduleForcesY) {}

    private List<TrajectoryPoint> getSolution(Variable T, VariableMatrix X, VariableMatrix U, int N) {
        List<TrajectoryPoint> trajectory = new ArrayList<>();
        double dt = T.value() / N;
        double mass = config.massKg();
        double moi = config.moi();
        double avgRadius = config.getDriveBaseRadius();

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
        if (path == null || path.isEmpty()) return null;
        List<SwerveSample> samples = new ArrayList<>();
        for (TrajectoryPoint p : path) {
            samples.add(new SwerveSample(p.time, p.x, p.y, p.theta, p.vx, p.vy, p.omega, p.ax, p.ay, p.alpha, p.moduleForcesX, p.moduleForcesY));
        }
        return new Trajectory<>("ProPath", samples, List.of(), List.of());
    }
}