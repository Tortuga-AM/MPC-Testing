package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone that invokes robot-thread callbacks when the actual robot enters,
 * exits, or remains inside the zone.
 *
 * Call update(robotPosition) once per AutoAlign periodic loop.
 *
 * Never run expensive planning, blocking I/O, or long-running code inside
 * callbacks. Callbacks run on the normal robot periodic thread.
 */
public class ActionableZone implements Zone {
    private final String name;
    private final Zone shape;

    private final Runnable onEnter;
    private final Runnable onExit;
    private final Runnable whileInside;

    private final Debouncer whileInsideDebouncer;

    private boolean wasInside = false;

    public ActionableZone(
        String name,
        Zone shape,
        Runnable onEnter,
        Runnable onExit,
        Runnable whileInside,
        double whileInsideDebounceSeconds
    ) {
        this.name = name;
        this.shape = shape;
        this.onEnter = onEnter == null ? () -> {} : onEnter;
        this.onExit = onExit == null ? () -> {} : onExit;
        this.whileInside = whileInside == null ? () -> {} : whileInside;

        whileInsideDebouncer = new Debouncer(
            Math.max(0.0, whileInsideDebounceSeconds)
        );
    }

    @Override
    public boolean contains(Translation2d position) {
        return shape.contains(position);
    }

    @Override
    public String getName() {
        return name;
    }

    /**
     * Evaluate this zone against the actual current robot position.
     */
    public void update(Translation2d robotPosition) {
        boolean inside = contains(robotPosition);

        if (inside && !wasInside) {
            onEnter.run();
        } else if (!inside && wasInside) {
            onExit.run();
        }

        if (whileInsideDebouncer.calculate(inside)) {
            whileInside.run();
        }

        wasInside = inside;
    }

    public boolean isCurrentlyInside() {
        return wasInside;
    }

    public void reset() {
        wasInside = false;
        whileInsideDebouncer.calculate(false);
    }
}