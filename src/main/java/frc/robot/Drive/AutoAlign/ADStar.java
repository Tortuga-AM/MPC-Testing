package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.littletonrobotics.junction.Logger;

/**
 * Custom AD*-style grid pathfinder.
 *
 * <p>Uses PathPlanner's navgrid.json format temporarily, but does not use any
 * PathPlanner classes at runtime.
 *
 * <p>Returned paths are:
 *
 * <ul>
 *   <li>rawGridPath: every grid cell selected by AD*
 *   <li>simplifiedPath: a line-of-sight-shortcut polyline in field coordinates
 * </ul>
 *
 * <p>The simplified path is still a polyline. Do not spline it yet without
 * collision-checking the resulting curve.
 */
public class ADStar {
    private static final double INITIAL_EPSILON = 2.5;
    private static final double EPSILON_STEP = 0.5;

    /*
     * Safety limit. A full FRC field at a 0.2 m resolution is usually far
     * below this number of cells.
     */
    private static final int MAX_PATH_EXTRACTION_STEPS = 500;

    /*
     * Defaults used only if navgrid.json cannot be loaded.
     */
    private final double nodeSizeMeters;
    private final int nodesX;
    private final int nodesY;

    /*
     * AD* state.
     *
     * These are only accessed by the planning background thread.
     */
    private final Map<GridPosition, Double> g = new HashMap<>();
    private final Map<GridPosition, Double> rhs = new HashMap<>();
    /*
    * Lazy-update priority queue.
    *
    * Every time a state receives a new key, a new queue entry is inserted.
    * Old entries remain in the queue and are discarded when popped if they no
    * longer match the current key stored in openKeys.
    */
    private final PriorityQueue<OpenEntry> openQueue =
        new PriorityQueue<>(
            (a, b) -> compareKeys(a.key(), b.key())
        );

    private final Map<GridPosition, Key> openKeys = new HashMap<>();
    private final Set<GridPosition> incons = new HashSet<>();
    private final Set<GridPosition> closed = new HashSet<>();

    /*
     * Obstacle state.
     *
     * Static obstacles come from navgrid.json.
     * Dynamic obstacles are supplied by your robot code.
     */
    private final Set<GridPosition> staticObstacles = new HashSet<>();
    private final Set<GridPosition> dynamicObstacles = new HashSet<>();
    private final Set<GridPosition> requestedObstacles = new HashSet<>();

    /*
     * Request data is written from the robot thread and read by the planning
     * thread, so it is guarded by requestLock.
     */
    private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

    private GridPosition requestedStart = new GridPosition(0, 0);
    private GridPosition requestedGoal = new GridPosition(0, 0);

    private Translation2d requestedRealStart = Translation2d.kZero;
    private Translation2d requestedRealGoal = Translation2d.kZero;

    private boolean resetRequested = true;
    private boolean minorUpdateRequested = true;
    private boolean majorUpdateRequested = true;

    /*
     * Result data is guarded by resultLock.
     */
    private final ReadWriteLock resultLock = new ReentrantReadWriteLock();

    private PathResult latestPath = PathResult.empty();

    private volatile boolean newPathAvailable = false;
    private volatile boolean running = true;
    private volatile double lastPlanningTimeSeconds = 0.0;

    private double epsilon = INITIAL_EPSILON;

    private final Thread planningThread;

    private volatile boolean lastStartWasProjected = false;
    private volatile Translation2d lastRequestedStart = Translation2d.kZero;
    private volatile Translation2d lastProjectedStart = Translation2d.kZero;

    private volatile boolean lastGoalWasProjected = false;
    private volatile Translation2d lastProjectedGoal = Translation2d.kZero;


    public ADStar(FieldObstacleMap fieldMap) {
        nodeSizeMeters = fieldMap.getNodeSizeMeters();
        nodesX = fieldMap.getNodesX();
        nodesY = fieldMap.getNodesY();

        /*
        * Copy static obstacles from the generated occupancy map once.
        */
        for (int x = 0; x < nodesX; x++) {
            for (int y = 0; y < nodesY; y++) {
                GridPosition position = new GridPosition(x, y);

                if (fieldMap.isOccupied(x, y)) {
                    staticObstacles.add(position);
                }
            }
        }

        requestedObstacles.addAll(staticObstacles);

        planningThread = new Thread(
            this::runPlanningThread,
            "Custom AD* Planning Thread"
        );

        planningThread.setDaemon(true);
        planningThread.start();
    }

    /**
     * Stop the background planning thread.
     *
     * <p>You normally do not need this on a robot, but it is useful for unit
     * tests or simulation cleanup.
     */
    public void close() {
        running = false;
        planningThread.interrupt();
    }

    /**
     * @return true once whenever a newly planned path has been published.
     */
    public boolean isNewPathAvailable() {
        return newPathAvailable;
    }

    /**
     * Returns the latest completed path.
     *
     * <p>This also consumes the "new path available" flag.
     */
    public PathResult getLatestPath() {
        resultLock.readLock().lock();

        try {
            newPathAvailable = false;
            return latestPath;
        } finally {
            resultLock.readLock().unlock();
        }
    }

    /**
     * Returns the latest path without consuming the "new path available" flag.
     */
    public PathResult peekLatestPath() {
        resultLock.readLock().lock();

        try {
            return latestPath;
        } finally {
            resultLock.readLock().unlock();
        }
    }

    public double getLastPlanningTimeSeconds() {
        return lastPlanningTimeSeconds;
    }

    public double getNodeSizeMeters() {
        return nodeSizeMeters;
    }

    /**
     * Requests a new start position.
     *
     * <p>If the requested point is inside an obstacle or outside the grid, it
     * is moved to the nearest free grid cell.
     */
    public void setStartPosition(Translation2d startPosition) {
        requestLock.writeLock().lock();

        try {
            GridPosition requestedGrid = toGridPosition(startPosition);

            GridPosition gridStart = findClosestFree(
                requestedGrid,
                requestedObstacles
            );

            if (gridStart == null) {
                return;
            }

            Translation2d projectedStart = toFieldPosition(gridStart);

            lastRequestedStart = startPosition;
            lastProjectedStart = projectedStart;

            lastStartWasProjected =
                !isInBounds(requestedGrid)
                    || requestedObstacles.contains(requestedGrid);

            /*
            * If the actual start is blocked, use the nearest valid point as the
            * trajectory start. This prevents the collision checker from rejecting
            * the path because its first point is known to be occupied.
            */
            requestedRealStart = lastStartWasProjected
                ? projectedStart
                : startPosition;

            if (!gridStart.equals(requestedStart)) {
                requestedStart = gridStart;
                minorUpdateRequested = true;
                newPathAvailable = false;
            }
        } finally {
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Requests a new goal position.
     *
     * <p>A changed goal resets AD* because the goal's rhs value is the root of
     * the reverse search.
     */
    public void setGoalPosition(Translation2d goalPosition) {
        requestLock.writeLock().lock();

        try {
            GridPosition requestedGrid = toGridPosition(goalPosition);

            GridPosition gridGoal = findClosestFree(
                requestedGrid,
                requestedObstacles
            );

            if (gridGoal == null) {
                return;
            }

            Translation2d projectedGoal = toFieldPosition(gridGoal);

            lastGoalWasProjected =
                !isInBounds(requestedGrid)
                    || requestedObstacles.contains(requestedGrid);

            lastProjectedGoal = projectedGoal;

            requestedGoal = gridGoal;

            /*
            * Same policy as the start:
            * if target is blocked, plan to nearest safe grid-cell center.
            */
            requestedRealGoal = lastGoalWasProjected
                ? projectedGoal
                : goalPosition;

            resetRequested = true;
            minorUpdateRequested = true;
            majorUpdateRequested = true;
            newPathAvailable = false;
        } finally {
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Replace all dynamic rectangular obstacles.
     *
     * <p>Each obstacle must already be inflated for:
     *
     * <ul>
     *   <li>robot bumper footprint
     *   <li>localization uncertainty
     *   <li>desired safety margin
     *   <li>opponent motion uncertainty, if applicable
     * </ul>
     *
     * <p>If the currently returned raw grid path intersects an obstacle, this
     * requests a new route from currentRobotPosition to the current goal.
     */
    public void setDynamicObstacles(
        List<ObstacleRectangle> obstacles,
        Translation2d currentRobotPosition
    ) {
        Set<GridPosition> newDynamicObstacles = new HashSet<>();

        for (ObstacleRectangle obstacle : obstacles) {
            GridPosition first = toGridPosition(obstacle.firstCorner());
            GridPosition second = toGridPosition(obstacle.secondCorner());

            int minX = Math.max(0, Math.min(first.x(), second.x()));
            int maxX = Math.min(nodesX - 1, Math.max(first.x(), second.x()));

            int minY = Math.max(0, Math.min(first.y(), second.y()));
            int maxY = Math.min(nodesY - 1, Math.max(first.y(), second.y()));

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    newDynamicObstacles.add(new GridPosition(x, y));
                }
            }
        }

        boolean currentPathBlocked;

        resultLock.readLock().lock();
        try {
            currentPathBlocked = latestPath.rawGridPath().stream()
                .anyMatch(newDynamicObstacles::contains);
        } finally {
            resultLock.readLock().unlock();
        }

        requestLock.writeLock().lock();

        try {
            dynamicObstacles.clear();
            dynamicObstacles.addAll(newDynamicObstacles);

            requestedObstacles.clear();
            requestedObstacles.addAll(staticObstacles);
            requestedObstacles.addAll(dynamicObstacles);

            if (currentPathBlocked) {
                GridPosition newStart = findClosestFree(
                    toGridPosition(currentRobotPosition),
                    requestedObstacles
                );

                GridPosition newGoal = findClosestFree(
                    toGridPosition(requestedRealGoal),
                    requestedObstacles
                );

                if (newStart != null && newGoal != null) {
                    requestedStart = newStart;
                    requestedRealStart = currentRobotPosition;
                    requestedGoal = newGoal;

                    resetRequested = true;
                    minorUpdateRequested = true;
                    majorUpdateRequested = true;
                    newPathAvailable = false;
                }
            }
        } finally {
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Call this from AutoAlign.periodic(), not from the planner's background
     * thread. This makes the planner easy to inspect in AdvantageScope.
     */
    public void logToAdvantageKit(String key) {
        PathResult path = peekLatestPath();

        Logger.recordOutput(key + "/PathValid", path.valid());
        Logger.recordOutput(key + "/NewPathAvailable", newPathAvailable);
        Logger.recordOutput(key + "/Epsilon", path.epsilon());
        Logger.recordOutput(
            key + "/LastPlanningTimeSeconds",
            lastPlanningTimeSeconds
        );

        Logger.recordOutput(
            key + "/RawGridPath",
            flattenGridPath(path.rawGridPath())
        );

        Logger.recordOutput(
            key + "/SimplifiedPath",
            flattenTranslationPath(path.simplifiedPath())
        );

        Logger.recordOutput(
            key + "/StaticObstacles",
            flattenGridPath(new ArrayList<>(staticObstacles))
        );

        Logger.recordOutput(
            key + "/DynamicObstacles",
            flattenGridPath(new ArrayList<>(dynamicObstacles))
        );
    }

    // =========================================================================
    // Background Planning Thread
    // =========================================================================

    private void runPlanningThread() {
        while (running) {
            PlanningRequest request = copyPlanningRequest();

            if (!request.hasWork()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException ignored) {
                    // Wake up and re-check running/request state.
                }

                continue;
            }

            double startTimeSeconds = System.nanoTime() * 1e-9;

            try {
                if (request.reset()) {
                    reset(request.start(), request.goal());
                }

                if (request.minorUpdate()) {
                    computeOrImprovePath(
                        request.start(),
                        request.goal(),
                        request.obstacles()
                    );

                    publishPath(request);
                } else if (request.majorUpdate() && epsilon > 1.0) {
                    epsilon = Math.max(1.0, epsilon - EPSILON_STEP);

                    /*
                     * AD* / ARA* refinement:
                     * inconsistent states return to OPEN with new keys.
                     */
                    for (GridPosition state : incons) {
                        addToOpen(state, key(state, request.start()));
                    }

                    incons.clear();
                    closed.clear();

                    computeOrImprovePath(
                        request.start(),
                        request.goal(),
                        request.obstacles()
                    );

                    publishPath(request);
                }
            } catch (Exception exception) {
                /*
                 * Do not crash the robot because a planning iteration failed.
                 * Request a clean reset on the next planner loop.
                 */
                requestLock.writeLock().lock();

                try {
                    resetRequested = true;
                    minorUpdateRequested = true;
                    majorUpdateRequested = true;
                } finally {
                    requestLock.writeLock().unlock();
                }
            }

            lastPlanningTimeSeconds =
                (System.nanoTime() * 1e-9) - startTimeSeconds;
        }
    }

    private PlanningRequest copyPlanningRequest() {
        requestLock.writeLock().lock();

        try {
            boolean doReset = resetRequested;
            boolean doMinor = minorUpdateRequested;
            boolean doMajor = !doMinor && majorUpdateRequested && epsilon > 1.0;

            PlanningRequest request = new PlanningRequest(
                doReset,
                doMinor,
                doMajor,
                requestedStart,
                requestedGoal,
                requestedRealStart,
                requestedRealGoal,
                new HashSet<>(requestedObstacles)
            );

            if (doReset) {
                resetRequested = false;
            }

            if (doMinor) {
                minorUpdateRequested = false;
            } else if (doMajor && epsilon - EPSILON_STEP <= 1.0) {
                /*
                 * The current major update will perform the final epsilon = 1
                 * refinement. Do not request another afterward.
                 */
                majorUpdateRequested = false;
            }

            return request;
        } finally {
            requestLock.writeLock().unlock();
        }
    }

    private void publishPath(PlanningRequest request) {
        List<GridPosition> rawPath = extractPath(
            request.start(),
            request.goal(),
            request.obstacles()
        );

        List<Translation2d> simplifiedPath = simplifyPath(
            rawPath,
            request.realStart(),
            request.realGoal(),
            request.obstacles()
        );

        PathResult result = new PathResult(
            !simplifiedPath.isEmpty(),
            rawPath,
            simplifiedPath,
            epsilon
        );

        resultLock.writeLock().lock();

        try {
            latestPath = result;
            newPathAvailable = true;
        } finally {
            resultLock.writeLock().unlock();
        }
    }

    // =========================================================================
    // AD* Core
    // =========================================================================

    private void reset(GridPosition start, GridPosition goal) {
        g.clear();
        rhs.clear();
        openQueue.clear();
        openKeys.clear();
        incons.clear();
        closed.clear();

        for (int x = 0; x < nodesX; x++) {
            for (int y = 0; y < nodesY; y++) {
                GridPosition position = new GridPosition(x, y);

                g.put(position, Double.POSITIVE_INFINITY);
                rhs.put(position, Double.POSITIVE_INFINITY);
            }
        }

        rhs.put(goal, 0.0);

        epsilon = INITIAL_EPSILON;
        addToOpen(goal, key(goal, start));
    }

    private void computeOrImprovePath(
        GridPosition start,
        GridPosition goal,
        Set<GridPosition> obstacles
    ) {
        while (true) {
            Map.Entry<GridPosition, Key> topEntry = getTopOpenEntry();

            if (topEntry == null) {
                return;
            }

            GridPosition state = topEntry.getKey();
            Key topKey = topEntry.getValue();

            if (compareKeys(topKey, key(start, start)) >= 0
                && equalCosts(rhs.get(start), g.get(start))) {
                return;
            }

            popTopOpenEntry();

            if (g.get(state) > rhs.get(state)) {
                g.put(state, rhs.get(state));
                closed.add(state);

                for (GridPosition neighbor : getOpenNeighbors(state, obstacles)) {
                    updateState(neighbor, start, goal, obstacles);
                }
            } else {
                g.put(state, Double.POSITIVE_INFINITY);

                for (GridPosition neighbor : getOpenNeighbors(state, obstacles)) {
                    updateState(neighbor, start, goal, obstacles);
                }

                updateState(state, start, goal, obstacles);
            }
        }
    }

    private void updateState(
        GridPosition state,
        GridPosition start,
        GridPosition goal,
        Set<GridPosition> obstacles
    ) {
        if (!state.equals(goal)) {
            double bestRhs = Double.POSITIVE_INFINITY;

            for (GridPosition neighbor : getOpenNeighbors(state, obstacles)) {
                double candidate =
                    g.get(neighbor) + movementCost(state, neighbor, obstacles);

                bestRhs = Math.min(bestRhs, candidate);
            }

            rhs.put(state, bestRhs);
        }

        removeFromOpen(state);
        if (!equalCosts(g.get(state), rhs.get(state))) {
            if (closed.contains(state)) {
                incons.add(state);
            } else {
                addToOpen(state, key(state, start));
            }
        }
    }

    private List<GridPosition> extractPath(
        GridPosition start,
        GridPosition goal,
        Set<GridPosition> obstacles
    ) {
        if (start.equals(goal)) {
            return List.of(start);
        }

        if (Double.isInfinite(g.getOrDefault(start, Double.POSITIVE_INFINITY))) {
            return List.of();
        }

        List<GridPosition> path = new ArrayList<>();
        Set<GridPosition> visited = new HashSet<>();

        GridPosition current = start;
        path.add(current);

        for (
            int step = 0;
            step < MAX_PATH_EXTRACTION_STEPS;
            step++
        ) {
            if (current.equals(goal)) {
                return path;
            }

            visited.add(current);

            GridPosition bestNeighbor = null;
            double bestCost = Double.POSITIVE_INFINITY;

            for (GridPosition neighbor : getOpenNeighbors(current, obstacles)) {
                if (visited.contains(neighbor)) {
                    continue;
                }

                double cost =
                    g.get(neighbor) + movementCost(current, neighbor, obstacles);

                if (cost < bestCost) {
                    bestCost = cost;
                    bestNeighbor = neighbor;
                }
            }

            if (bestNeighbor == null || Double.isInfinite(bestCost)) {
                return List.of();
            }

            current = bestNeighbor;
            path.add(current);
        }

        /*
         * If we hit this, something is likely wrong with the extracted path.
         * Return invalid rather than letting the robot follow a partial route.
         */
        return List.of();
    }

    // =========================================================================
    // Path Simplification
    // =========================================================================

    private List<Translation2d> simplifyPath(
        List<GridPosition> rawPath,
        Translation2d realStart,
        Translation2d realGoal,
        Set<GridPosition> obstacles
    ) {
        if (rawPath.size() < 2) {
            return List.of();
        }

        List<GridPosition> simplifiedGridPath = new ArrayList<>();
        simplifiedGridPath.add(rawPath.get(0));

        /*
         * Greedy line-of-sight simplification.
         *
         * If the last retained point can see the next point in the raw path,
         * the current point is unnecessary. Otherwise, retain it as a corner.
         */
        for (int i = 1; i < rawPath.size() - 1; i++) {
            GridPosition lastKept =
                simplifiedGridPath.get(simplifiedGridPath.size() - 1);

            if (!hasLineOfSight(lastKept, rawPath.get(i + 1), obstacles)) {
                simplifiedGridPath.add(rawPath.get(i));
            }
        }

        simplifiedGridPath.add(rawPath.get(rawPath.size() - 1));

        List<Translation2d> result = new ArrayList<>();

        for (GridPosition position : simplifiedGridPath) {
            result.add(toFieldPosition(position));
        }

        /*
         * Replace cell-center endpoints with the real requested positions.
         *
         * Note: later, add collision validation for these first/last segments,
         * especially if the real pose is near an obstacle boundary.
         */
        result.set(0, realStart);
        result.set(result.size() - 1, realGoal);

        return result;
    }

    /**
     * Bresenham-style collision check between two grid cells.
     */
    private boolean hasLineOfSight(
        GridPosition first,
        GridPosition second,
        Set<GridPosition> obstacles
    ) {
        int x = first.x();
        int y = first.y();

        int targetX = second.x();
        int targetY = second.y();

        int dx = Math.abs(targetX - x);
        int dy = Math.abs(targetY - y);

        int xStep = x < targetX ? 1 : -1;
        int yStep = y < targetY ? 1 : -1;

        int error = dx - dy;

        while (true) {
            if (obstacles.contains(new GridPosition(x, y))) {
                return false;
            }

            if (x == targetX && y == targetY) {
                return true;
            }

            int doubledError = 2 * error;

            if (doubledError > -dy) {
                error -= dy;
                x += xStep;
            }

            if (doubledError < dx) {
                error += dx;
                y += yStep;
            }
        }
    }

    // =========================================================================
    // Grid / Collision Helpers
    // =========================================================================

    private List<GridPosition> getOpenNeighbors(
        GridPosition state,
        Set<GridPosition> obstacles
    ) {
        List<GridPosition> neighbors = new ArrayList<>(8);

        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                /*
                 * Important: do not include the state itself as a neighbor.
                 * PathPlanner's copied implementation does include it, which
                 * produces an unnecessary zero-cost self edge.
                 */
                if (xMove == 0 && yMove == 0) {
                    continue;
                }

                GridPosition next = new GridPosition(
                    state.x() + xMove,
                    state.y() + yMove
                );

                if (isInBounds(next) && !obstacles.contains(next)) {
                    neighbors.add(next);
                }
            }
        }

        return neighbors;
    }

    private List<GridPosition> getAllNeighbors(GridPosition state) {
        List<GridPosition> neighbors = new ArrayList<>(8);

        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                if (xMove == 0 && yMove == 0) {
                    continue;
                }

                GridPosition next = new GridPosition(
                    state.x() + xMove,
                    state.y() + yMove
                );

                if (isInBounds(next)) {
                    neighbors.add(next);
                }
            }
        }

        return neighbors;
    }

    private double movementCost(
        GridPosition first,
        GridPosition second,
        Set<GridPosition> obstacles
    ) {
        if (hasEdgeCollision(first, second, obstacles)) {
            return Double.POSITIVE_INFINITY;
        }

        return heuristic(first, second);
    }

    /**
     * Rejects a diagonal move if it would pass through the corner between two
     * obstacle cells.
     */
    private boolean hasEdgeCollision(
        GridPosition first,
        GridPosition second,
        Set<GridPosition> obstacles
    ) {
        if (obstacles.contains(first) || obstacles.contains(second)) {
            return true;
        }

        boolean diagonal =
            first.x() != second.x() && first.y() != second.y();

        if (!diagonal) {
            return false;
        }

        GridPosition sideA = new GridPosition(first.x(), second.y());
        GridPosition sideB = new GridPosition(second.x(), first.y());

        return obstacles.contains(sideA) || obstacles.contains(sideB);
    }

    private GridPosition findClosestFree(
        GridPosition requested,
        Set<GridPosition> obstacles
    ) {
        GridPosition clamped = new GridPosition(
            Math.max(0, Math.min(nodesX - 1, requested.x())),
            Math.max(0, Math.min(nodesY - 1, requested.y()))
        );

        if (!obstacles.contains(clamped)) {
            return clamped;
        }

        Queue<GridPosition> queue = new ArrayDeque<>();
        Set<GridPosition> visited = new HashSet<>();

        queue.add(clamped);
        visited.add(clamped);

        while (!queue.isEmpty()) {
            GridPosition current = queue.remove();

            if (!obstacles.contains(current)) {
                return current;
            }

            for (GridPosition neighbor : getAllNeighbors(current)) {
                if (visited.add(neighbor)) {
                    queue.add(neighbor);
                }
            }
        }

        return null;
    }

    private boolean isInBounds(GridPosition position) {
        return position.x() >= 0
            && position.x() < nodesX
            && position.y() >= 0
            && position.y() < nodesY;
    }

    private GridPosition toGridPosition(Translation2d position) {
        return new GridPosition(
            (int) Math.floor(position.getX() / nodeSizeMeters),
            (int) Math.floor(position.getY() / nodeSizeMeters)
        );
    }

    private Translation2d toFieldPosition(GridPosition position) {
        return new Translation2d(
            (position.x() + 0.5) * nodeSizeMeters,
            (position.y() + 0.5) * nodeSizeMeters
        );
    }

    // =========================================================================
    // AD* Key Helpers
    // =========================================================================

    private Key key(GridPosition state, GridPosition start) {
        double gValue = g.getOrDefault(state, Double.POSITIVE_INFINITY);
        double rhsValue = rhs.getOrDefault(state, Double.POSITIVE_INFINITY);

        double minimum = Math.min(gValue, rhsValue);

        return new Key(
            minimum + epsilon * heuristic(start, state),
            minimum
        );
    }

    private int compareKeys(Key first, Key second) {
        int primary = Double.compare(first.primary(), second.primary());

        if (primary != 0) {
            return primary;
        }

        return Double.compare(first.secondary(), second.secondary());
    }

    private double heuristic(GridPosition first, GridPosition second) {
        return Math.hypot(
            second.x() - first.x(),
            second.y() - first.y()
        );
    }

    private boolean equalCosts(double first, double second) {
        if (Double.isInfinite(first) && Double.isInfinite(second)) {
            return true;
        }

        return Math.abs(first - second) < 1e-9;
    }

    // =========================================================================
    // Logging Helpers
    // =========================================================================

    private double[] flattenTranslationPath(List<Translation2d> path) {
        double[] output = new double[path.size() * 2];

        for (int i = 0; i < path.size(); i++) {
            output[i * 2] = path.get(i).getX();
            output[i * 2 + 1] = path.get(i).getY();
        }

        return output;
    }

    private double[] flattenGridPath(List<GridPosition> path) {
        double[] output = new double[path.size() * 2];

        for (int i = 0; i < path.size(); i++) {
            Translation2d fieldPosition = toFieldPosition(path.get(i));

            output[i * 2] = fieldPosition.getX();
            output[i * 2 + 1] = fieldPosition.getY();
        }

        return output;
    }

    // =========================================================================
    // Public Data Types
    // =========================================================================

    public record GridPosition(int x, int y) {}

    /**
     * A rectangular obstacle specified by two opposite field-coordinate corners.
     */
    public record ObstacleRectangle(
        Translation2d firstCorner,
        Translation2d secondCorner
    ) {}

    /**
     * Immutable pathfinder result.
     */
    public record PathResult(
        boolean valid,
        List<GridPosition> rawGridPath,
        List<Translation2d> simplifiedPath,
        double epsilon
    ) {
        public PathResult {
            rawGridPath = List.copyOf(rawGridPath);
            simplifiedPath = List.copyOf(simplifiedPath);
        }

        public static PathResult empty() {
            return new PathResult(
                false,
                Collections.emptyList(),
                Collections.emptyList(),
                INITIAL_EPSILON
            );
        }
    }

    private record Key(double primary, double secondary) {}

    private record PlanningRequest(
        boolean reset,
        boolean minorUpdate,
        boolean majorUpdate,
        GridPosition start,
        GridPosition goal,
        Translation2d realStart,
        Translation2d realGoal,
        Set<GridPosition> obstacles
    ) {
        boolean hasWork() {
            return reset || minorUpdate || majorUpdate;
        }
    }

    public boolean wasLastStartProjected() {
        return lastStartWasProjected;
    }

    public Translation2d getLastProjectedStart() {
        return lastProjectedStart;
    }

    public Translation2d getLastRequestedStart() {
        return lastRequestedStart;
    }
    private void addToOpen(GridPosition state, Key key) {
        openKeys.put(state, key);
        openQueue.add(new OpenEntry(state, key));
    }

    private Map.Entry<GridPosition, Key> getTopOpenEntry() {
        while (!openQueue.isEmpty()) {
            OpenEntry entry = openQueue.peek();
            Key currentKey = openKeys.get(entry.state());

            if (currentKey == null || compareKeys(currentKey, entry.key()) != 0) {
                openQueue.poll();
                continue;
            }

            return Map.entry(entry.state(), entry.key());
        }

        return null;
    }

    private Map.Entry<GridPosition, Key> popTopOpenEntry() {
        while (!openQueue.isEmpty()) {
            OpenEntry entry = openQueue.poll();
            Key currentKey = openKeys.get(entry.state());

            if (currentKey == null || compareKeys(currentKey, entry.key()) != 0) {
                continue;
            }

            openKeys.remove(entry.state());
            return Map.entry(entry.state(), entry.key());
        }

        return null;
    }

    public boolean wasLastGoalProjected() {
        return lastGoalWasProjected;
    }

    public Translation2d getLastProjectedGoal() {
        return lastProjectedGoal;
    }

    private record OpenEntry(GridPosition state, Key key) {}

    private void removeFromOpen(GridPosition state) {
        /*
        * Do not remove from PriorityQueue directly; PriorityQueue.remove(Object)
        * is O(n). Instead remove the authoritative key from openKeys.
        *
        * Any older queue entry becomes stale and will be discarded by
        * getTopOpenEntry() / popTopOpenEntry().
        */
        openKeys.remove(state);
    }
}
