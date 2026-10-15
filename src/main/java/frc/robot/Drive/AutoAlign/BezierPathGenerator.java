package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Converts a collision-free polyline into cubic Bezier segments.
 *
 * <p>This does not perform collision checking itself. The caller must validate
 * generated segments against the occupancy grid before following them.
 */
public class BezierPathGenerator {
    /*
     * Similar in spirit to PathPlanner's smoothing anchor percentage.
     *
     * Lower value:
     * - anchors stay farther from corners
     * - gentler, larger turns
     *
     * Higher value:
     * - anchors move closer to corners
     * - tighter turns
     *
     * Start around 0.70 to 0.80.
     */
    private final double cornerAnchorPercent;

    /*
     * Limits how much of a neighboring segment a corner may consume.
     * 0.40 means each anchor remains at least 40% of each local segment away
     * from the adjacent waypoint.
     */
    private final double maxCornerCutPercent;

    private final double controlHandleScale;

    public BezierPathGenerator() {
        this(0.80, 0.40, 1.0);
    }

    public BezierPathGenerator(
        double cornerAnchorPercent,
        double maxCornerCutPercent
    ) {
        this(cornerAnchorPercent, maxCornerCutPercent, 1.0);
    }

    public BezierPathGenerator(
        double cornerAnchorPercent,
        double maxCornerCutPercent,
        double controlHandleScale
    ) {
        if (cornerAnchorPercent <= 0.0 || cornerAnchorPercent >= 1.0) {
            throw new IllegalArgumentException(
                "cornerAnchorPercent must be between 0 and 1"
            );
        }

        if (maxCornerCutPercent <= 0.0 || maxCornerCutPercent >= 0.5) {
            throw new IllegalArgumentException(
                "maxCornerCutPercent must be greater than 0 and less than 0.5"
            );
        }

        if (controlHandleScale < 0.0 || controlHandleScale > 1.0) {
            throw new IllegalArgumentException(
                "controlHandleScale must be between 0 and 1"
            );
        }

        this.cornerAnchorPercent = cornerAnchorPercent;
        this.maxCornerCutPercent = maxCornerCutPercent;
        this.controlHandleScale = controlHandleScale;
    }

    /**
     * Generates C1-continuous-ish cubic Bezier segments.
     *
     * @param polyline Simplified AD* points: start, optional corners, goal
     * @return Bezier segments. Empty if fewer than two points are supplied.
     */
    public List<CubicBezierSegment> generate(
        List<Translation2d> polyline
    ) {
        if (polyline == null || polyline.size() < 2) {
            return List.of();
        }

        if (polyline.size() == 2) {
            return List.of(createStraightSegment(
                polyline.get(0),
                polyline.get(1)
            ));
        }

        /*
         * These are the actual anchors that Bézier segments connect.
         *
         * A polyline:
         * start -> corner -> goal
         *
         * becomes:
         * start -> anchorBeforeCorner -> anchorAfterCorner -> goal
         */
        List<Translation2d> anchors = new ArrayList<>();
        anchors.add(polyline.get(0));

        for (int i = 1; i < polyline.size() - 1; i++) {
            Translation2d previous = polyline.get(i - 1);
            Translation2d corner = polyline.get(i);
            Translation2d next = polyline.get(i + 1);

            double previousDistance = previous.getDistance(corner);
            double nextDistance = corner.getDistance(next);

            if (previousDistance < 1e-6 || nextDistance < 1e-6) {
                continue;
            }

            /*
             * Move from corner toward previous / next to form two anchors.
             *
             * PathPlanner's basic concept is:
             * anchorBefore = corner - 80% of incoming vector
             * anchorAfter  = corner - 80% of outgoing reverse vector
             *
             * The local max cut avoids neighboring corner smoothing overlaps.
             */
            double maxCutDistance = Math.min(
                previousDistance * maxCornerCutPercent,
                nextDistance * maxCornerCutPercent
            );

            double requestedCutDistance = Math.min(
                previousDistance * (1.0 - cornerAnchorPercent),
                nextDistance * (1.0 - cornerAnchorPercent)
            );

            double cutDistance = Math.min(
                requestedCutDistance,
                maxCutDistance
            );

            Translation2d towardPrevious = previous
                .minus(corner)
                .div(previousDistance);

            Translation2d towardNext = next
                .minus(corner)
                .div(nextDistance);

            Translation2d anchorBefore = corner.plus(
                towardPrevious.times(cutDistance)
            );

            Translation2d anchorAfter = corner.plus(
                towardNext.times(cutDistance)
            );

            anchors.add(anchorBefore);
            anchors.add(anchorAfter);
        }

        anchors.add(polyline.get(polyline.size() - 1));

        List<CubicBezierSegment> segments = new ArrayList<>();

        for (int i = 0; i < anchors.size() - 1; i++) {
            Translation2d startAnchor = anchors.get(i);
            Translation2d endAnchor = anchors.get(i + 1);

            /*
             * The simple first implementation uses each anchor's local
             * neighboring direction as the control tangent.
             */
            Translation2d startDirection = getAnchorDirection(
                anchors,
                i,
                true
            );

            Translation2d endDirection = getAnchorDirection(
                anchors,
                i + 1,
                false
            );

            double anchorDistance = startAnchor.getDistance(endAnchor);

            /*
             * Cubic control handle length. One-third is a good initial
             * straight-ish segment value. Reduce this later when collision
             * checking finds curve bulging near obstacles.
             */
            double handleLength = (anchorDistance / 3.0) * controlHandleScale;
            Translation2d p1 = startAnchor.plus(
                startDirection.times(handleLength)
            );

            Translation2d p2 = endAnchor.minus(
                endDirection.times(handleLength)
            );

            segments.add(new CubicBezierSegment(
                startAnchor,
                p1,
                p2,
                endAnchor
            ));
        }

        return segments;
    }

    private CubicBezierSegment createStraightSegment(
        Translation2d start,
        Translation2d end
    ) {
        Translation2d delta = end.minus(start);

        return new CubicBezierSegment(
            start,
            start.plus(delta.times(1.0 / 3.0)),
            start.plus(delta.times(2.0 / 3.0)),
            end
        );
    }

    private Translation2d getAnchorDirection(
        List<Translation2d> anchors,
        int anchorIndex,
        boolean forward
    ) {
        Translation2d direction;

        if (anchorIndex == 0) {
            direction = anchors.get(1).minus(anchors.get(0));
        } else if (anchorIndex == anchors.size() - 1) {
            direction = anchors.get(anchorIndex)
                .minus(anchors.get(anchorIndex - 1));
        } else {
            /*
             * Average-ish tangent across neighboring anchors.
             */
            direction = anchors.get(anchorIndex + 1)
                .minus(anchors.get(anchorIndex - 1));
        }

        if (direction.getNorm() < 1e-9) {
            return forward
                ? new Translation2d(1.0, 0.0)
                : new Translation2d(1.0, 0.0);
        }

        return direction.div(direction.getNorm());
    }
}