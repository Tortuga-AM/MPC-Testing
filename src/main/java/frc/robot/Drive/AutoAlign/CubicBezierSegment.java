package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A 2D cubic Bezier segment.
 *
 * P0/P3 are anchors. P1/P2 are control points.
 */
public class CubicBezierSegment {
    private final Translation2d p0;
    private final Translation2d p1;
    private final Translation2d p2;
    private final Translation2d p3;

    public CubicBezierSegment(
        Translation2d p0,
        Translation2d p1,
        Translation2d p2,
        Translation2d p3
    ) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
    }

    public Translation2d getP0() {
        return p0;
    }

    public Translation2d getP1() {
        return p1;
    }

    public Translation2d getP2() {
        return p2;
    }

    public Translation2d getP3() {
        return p3;
    }

    /**
     * Position at t, where t is clamped from 0 to 1.
     */
    public Translation2d getPosition(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        double oneMinusT = 1.0 - t;

        double p0Weight = oneMinusT * oneMinusT * oneMinusT;
        double p1Weight = 3.0 * oneMinusT * oneMinusT * t;
        double p2Weight = 3.0 * oneMinusT * t * t;
        double p3Weight = t * t * t;

        return p0.times(p0Weight)
            .plus(p1.times(p1Weight))
            .plus(p2.times(p2Weight))
            .plus(p3.times(p3Weight));
    }

    /**
     * First derivative with respect to t.
     *
     * The derivative direction is the path tangent.
     */
    public Translation2d getFirstDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        double oneMinusT = 1.0 - t;

        Translation2d term0 = p1.minus(p0).times(
            3.0 * oneMinusT * oneMinusT
        );

        Translation2d term1 = p2.minus(p1).times(
            6.0 * oneMinusT * t
        );

        Translation2d term2 = p3.minus(p2).times(
            3.0 * t * t
        );

        return term0.plus(term1).plus(term2);
    }

    /**
     * Second derivative with respect to t.
     *
     * Used to calculate curvature.
     */
    public Translation2d getSecondDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        double oneMinusT = 1.0 - t;

        Translation2d term0 = p2
            .minus(p1.times(2.0))
            .plus(p0)
            .times(6.0 * oneMinusT);

        Translation2d term1 = p3
            .minus(p2.times(2.0))
            .plus(p1)
            .times(6.0 * t);

        return term0.plus(term1);
    }

    public Rotation2d getTangent(double t) {
        Translation2d derivative = getFirstDerivative(t);

        if (derivative.getNorm() < 1e-9) {
            return Rotation2d.kZero;
        }

        return derivative.getAngle();
    }

    /**
     * Signed curvature:
     *
     * kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
     */
    public double getCurvature(double t) {
        Translation2d firstDerivative = getFirstDerivative(t);
        Translation2d secondDerivative = getSecondDerivative(t);

        double dx = firstDerivative.getX();
        double dy = firstDerivative.getY();

        double ddx = secondDerivative.getX();
        double ddy = secondDerivative.getY();

        double denominator = Math.pow(
            (dx * dx) + (dy * dy),
            1.5
        );

        if (denominator < 1e-9) {
            return 0.0;
        }

        return ((dx * ddy) - (dy * ddx)) / denominator;
    }

    public PathSample sample(double t) {
        return new PathSample(
            getPosition(t),
            getTangent(t),
            getCurvature(t)
        );
    }

    /**
     * Approximate arc length by subdividing the curve.
     *
     * Use more samples for more accurate timing. 50 is a reasonable initial
     * value for FRC-scale curves.
     */
    public double approximateLength(int samples) {
        if (samples < 1) {
            throw new IllegalArgumentException(
                "Bezier length requires at least one sample"
            );
        }

        double length = 0.0;
        Translation2d previous = getPosition(0.0);

        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            Translation2d current = getPosition(t);

            length += current.getDistance(previous);
            previous = current;
        }

        return length;
    }
}