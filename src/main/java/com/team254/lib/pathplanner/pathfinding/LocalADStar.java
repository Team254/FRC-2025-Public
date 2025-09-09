package com.team254.lib.pathplanner.pathfinding;

import com.team254.lib.pathplanner.path.GoalEndState;
import com.team254.lib.pathplanner.path.IdealStartingState;
import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import com.team254.lib.pathplanner.path.Waypoint;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* running locally in a background thread
 *
 * <p>I would like to apologize to anyone trying to understand this code. The implementation I
 * translated it from was much worse.
 */
public class LocalADStar implements Pathfinder {
    private static final double SMOOTHING_ANCHOR_PCT = 0.8;
    // We don't need the anytime part.
    private static final double EPS = 1.0;

    // Cached paths.
    private static final double kMaxVelocityForBezierCache = 0.05;
    private static final int kMaxCacheSize = 120;
    private Map<Pair<GridPosition, GridPosition>, PathPlannerPath> pathCache =
            new HashMap<Pair<GridPosition, GridPosition>, PathPlannerPath>();

    private double fieldLength = 16.54;
    private double fieldWidth = 8.02;

    private double nodeSize = 0.2;

    private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
    private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

    private final HashMap<GridPosition, Double> g = new HashMap<>();
    private final HashMap<GridPosition, Double> rhs = new HashMap<>();
    private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();
    private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
    private final Set<GridPosition> closed = new HashSet<>();
    private final Set<GridPosition> teleopObstacles = new HashSet<>();
    private final Set<GridPosition> backoffObstacles = new HashSet<>();
    private final Set<GridPosition> autoObstacles = new HashSet<>();
    private final Set<GridPosition> dynamicObstacles = new HashSet<>();
    private Set<GridPosition> requestObstacles = new HashSet<>();

    private boolean enableCaching = false;
    private double cacheToleranceMeters = 0.8;
    private GridPosition requestStart;
    private Pose2d requestRealStartPose;
    private GridPosition requestGoal;
    private Pose2d requestRealGoalPose;
    private Translation2d requestStartVelocity;
    private PathConstraints requestConstraints;
    private double eps;
    private PathPlannerPath goalPath = null;

    private final Thread planningThread;
    private boolean requestMinor = true;
    private boolean requestReset = true;
    private boolean newPathAvailable = false;

    private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
    private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

    private List<GridPosition> currentPathFull = new ArrayList<>();

    private PathPlannerPath currentPath = null;

    /** Create a new pathfinder that runs AD* locally in a background thread */
    public LocalADStar() {
        planningThread = new Thread(this::runThread);

        requestStart = new GridPosition(0, 0);
        requestRealStartPose = Pose2d.kZero;
        requestGoal = new GridPosition(0, 0);
        requestRealGoalPose = Pose2d.kZero;
        requestStartVelocity = Translation2d.kZero;

        teleopObstacles.clear();
        dynamicObstacles.clear();
        autoObstacles.clear();
        backoffObstacles.clear();

        loadObstacles(autoObstacles, "pathplanner/auto_navgrid.json");
        loadObstacles(backoffObstacles, "pathplanner/backoff_navgrid.json");
        // Load the static one second since it will have the last values of
        // some member variables.
        loadObstacles(teleopObstacles, "pathplanner/navgrid.json");

        autoObstacles.addAll(dynamicObstacles);
        teleopObstacles.addAll(dynamicObstacles);
        backoffObstacles.addAll(dynamicObstacles);

        requestObstacles = teleopObstacles;

        requestReset = false;
        requestMinor = false;

        newPathAvailable = false;

        planningThread.setDaemon(true);
        planningThread.setName("ADStar Planning Thread");
        planningThread.start();
    }

    private void loadObstacles(Set<GridPosition> set, String path) {
        File navGridFile = new File(Filesystem.getDeployDirectory(), path);
        if (navGridFile.exists()) {
            try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
                JSONArray grid = (JSONArray) json.get("grid");
                nodesY = grid.size();
                for (int row = 0; row < grid.size(); row++) {
                    JSONArray rowArray = (JSONArray) grid.get(row);
                    if (row == 0) {
                        nodesX = rowArray.size();
                    }
                    for (int col = 0; col < rowArray.size(); col++) {
                        boolean isObstacle = (boolean) rowArray.get(col);
                        if (isObstacle) {
                            set.add(new GridPosition(col, row));
                        }
                    }
                }

                JSONObject fieldSize = (JSONObject) json.get("field_size");
                fieldLength = ((Number) fieldSize.get("x")).doubleValue();
                fieldWidth = ((Number) fieldSize.get("y")).doubleValue();

                System.out.println("nodeSize: " + nodeSize);
                System.out.println("nodesY: " + nodesY);
                System.out.println("nodeSX: " + nodesX);

                System.out.println("fieldLength: " + fieldLength);
                System.out.println("fieldWidth: " + fieldWidth);
                System.out.println("Successfully loaded navgrid: " + path);
            } catch (Exception e) {
                // Do nothing, use defaults
            }
        } else {
            System.out.println("Unable to load navgrid: " + path);
        }
    }

    @Override
    public void setAutoObstacles() {
        System.out.println("[PathPlanner] Using auto obstacles!");
        requestLock.writeLock().lock();
        requestObstacles = autoObstacles;
        requestLock.writeLock().unlock();
    }

    @Override
    public void setTeleopObstacles() {
        System.out.println("[PathPlanner] Using teleop obstacles!");
        requestLock.writeLock().lock();
        requestObstacles = teleopObstacles;
        requestLock.writeLock().unlock();
    }

    @Override
    public void setBackoffObstacles() {
        System.out.println("[PathPlanner] Using backoff obstacles!");
        requestLock.writeLock().lock();
        requestObstacles = backoffObstacles;
        requestLock.writeLock().unlock();
    }

    @Override
    public void enableCaching() {
        requestLock.writeLock().lock();
        enableCaching = true;
        requestLock.writeLock().unlock();
    }

    @Override
    public void disableCaching() {
        requestLock.writeLock().lock();
        enableCaching = false;
        requestLock.writeLock().unlock();
    }

    @Override
    public void setCacheDistanceToleranceMeters(double tolerance) {
        requestLock.writeLock().lock();
        cacheToleranceMeters = tolerance;
        requestLock.writeLock().unlock();
    }

    /**
     * Get if a new path has been calculated since the last time a path was retrieved
     *
     * @return True if a new path is available
     */
    @Override
    public boolean isNewPathAvailable() {
        boolean pathAvailable = false;

        requestLock.readLock().lock();
        pathAvailable = newPathAvailable;
        requestLock.readLock().unlock();

        return pathAvailable;
    }

    /**
     * Get the most recently calculated path
     *
     * @return The PathPlannerPath created from the points calculated by the pathfinder
     */
    @Override
    public PathPlannerPath getCurrentPath() {
        PathPlannerPath path = null;

        pathLock.readLock().lock();
        path = currentPath;
        pathLock.readLock().unlock();

        newPathAvailable = false;

        return path;
    }

    public void setProblem(
            Pose2d startPose,
            PathPlannerPath goalPath,
            Translation2d startVelocity,
            PathConstraints constraints) {
        requestLock.writeLock().lock();
        GridPosition startPos =
                findClosestNonObstacle(getGridPos(startPose.getTranslation()), requestObstacles);
        if (startPos != null) {
            requestStart = startPos;
            requestRealStartPose = startPose;
        }
        var goalPosition = goalPath.getPoint(0).position;
        GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);
        if (gridPos != null) {
            requestGoal = gridPos;
            requestRealGoalPose = goalPath.getPathPoses().get(0);
        }
        requestStartVelocity = startVelocity;
        requestConstraints = constraints;

        requestMinor = true;
        requestReset = true;
        newPathAvailable = false;
        this.goalPath = goalPath;
        requestLock.writeLock().unlock();
    }

    public void setProblem(
            Pose2d startPose,
            Pose2d goalPose,
            Translation2d startVelocity,
            PathConstraints constraints) {
        requestLock.writeLock().lock();
        GridPosition startPos =
                findClosestNonObstacle(getGridPos(startPose.getTranslation()), requestObstacles);
        if (startPos != null) {
            requestStart = startPos;
            requestRealStartPose = startPose;
        }
        GridPosition gridPos =
                findClosestNonObstacle(getGridPos(goalPose.getTranslation()), requestObstacles);
        if (gridPos != null) {
            requestGoal = gridPos;
            requestRealGoalPose = goalPose;
        }
        goalPath = null;
        requestStartVelocity = startVelocity;
        requestConstraints = constraints;

        requestMinor = true;
        requestReset = true;
        newPathAvailable = false;
        requestLock.writeLock().unlock();
    }

    /**
     * Set the start position to pathfind from
     *
     * @param startPosition Start position on the field. If this is within an obstacle it will be
     *     moved to the nearest non-obstacle node.
     */
    @Override
    public void setStartPose(Pose2d startPose) {
        GridPosition startPos =
                findClosestNonObstacle(getGridPos(startPose.getTranslation()), requestObstacles);

        if (startPos != null && !startPos.equals(requestStart)) {
            requestLock.writeLock().lock();
            requestStart = startPos;
            requestRealStartPose = startPose;

            requestMinor = true;
            newPathAvailable = false;
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Set the goal position to pathfind to
     *
     * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
     *     to the nearest non-obstacle node.
     */
    @Override
    public void setGoalPose(Pose2d goalPose) {
        GridPosition gridPos =
                findClosestNonObstacle(getGridPos(goalPose.getTranslation()), requestObstacles);

        if (gridPos != null) {
            requestLock.writeLock().lock();
            requestGoal = gridPos;
            requestRealGoalPose = goalPose;

            requestMinor = true;
            requestReset = true;
            newPathAvailable = false;
            requestLock.writeLock().unlock();
        }
    }

    @Override
    public void setStartVelocity(Translation2d startVelocity) {
        requestLock.writeLock().lock();
        try {
            requestStartVelocity = startVelocity;
        } finally {
            requestLock.writeLock().unlock();
        }
    }

    @Override
    public void setConstraints(PathConstraints constraints) {
        requestLock.writeLock().lock();
        try {
            requestConstraints = constraints;
        } finally {
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d
     *     represents opposite corners of a bounding box.
     * @param currentRobotPos The current position of the robot. This is needed to change the start
     *     position of the path if the robot is now within an obstacle.
     */
    @Override
    public void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs,
            Pose2d currentRobotPose,
            Translation2d startVelocity) {
        Set<GridPosition> newObs = new HashSet<>();

        for (var obstacle : obs) {
            var gridPos1 = getGridPos(obstacle.getFirst());
            var gridPos2 = getGridPos(obstacle.getSecond());

            int minX = Math.min(gridPos1.x, gridPos2.x);
            int maxX = Math.max(gridPos1.x, gridPos2.x);

            int minY = Math.min(gridPos1.y, gridPos2.y);
            int maxY = Math.max(gridPos1.y, gridPos2.y);

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    newObs.add(new GridPosition(x, y));
                }
            }
        }

        dynamicObstacles.clear();
        dynamicObstacles.addAll(newObs);
        requestLock.writeLock().lock();
        requestObstacles.addAll(dynamicObstacles);
        requestLock.writeLock().unlock();

        pathLock.readLock().lock();
        boolean recalculate = false;
        for (GridPosition pos : currentPathFull) {
            if (requestObstacles.contains(pos)) {
                recalculate = true;
                break;
            }
        }
        pathLock.readLock().unlock();

        if (recalculate) {
            setStartPose(currentRobotPose);
            setGoalPose(requestRealGoalPose);
            setStartVelocity(startVelocity);
        }
    }

    @SuppressWarnings("BusyWait")
    private void runThread() {
        while (true) {
            try {
                requestLock.readLock().lock();
                boolean reset = requestReset;
                boolean minor = requestMinor;

                // Change the request booleans based on what will be done this loop
                if (reset) {
                    requestReset = false;
                }

                if (minor) requestMinor = false;

                if (reset || minor) {
                    GridPosition start = requestStart;
                    Pose2d realStart = requestRealStartPose;
                    GridPosition goal = requestGoal;
                    Pose2d realGoal = requestRealGoalPose;
                    Set<GridPosition> obstacles = new HashSet<>(requestObstacles);
                    Translation2d startVelocity = requestStartVelocity;
                    PathConstraints constraints = requestConstraints;
                    requestLock.readLock().unlock();
                    Threads.setCurrentThreadPriority(true, 1);
                    doWork(
                            reset,
                            minor,
                            start,
                            goal,
                            realStart,
                            realGoal,
                            obstacles,
                            constraints,
                            startVelocity);
                    Threads.setCurrentThreadPriority(false, 5);
                } else {
                    requestLock.readLock().unlock();
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            } catch (Exception e) {
                // Something messed up. Reset and hope for the best
                requestLock.writeLock().lock();
                requestReset = true;
                requestLock.writeLock().unlock();
            }
        }
    }

    private PathPlannerPath checkCache(GridPosition start, GridPosition goal) {
        Pair<GridPosition, GridPosition> key = new Pair<GridPosition, GridPosition>(start, goal);
        // Always check the exact grid cell first.
        PathPlannerPath result = pathCache.getOrDefault(key, null);
        if (result != null) {
            System.out.println("[Cache] Exact cache hit lookup " + start + " -> " + key.getFirst());
            return result;
        }

        // Put the straight neighbors ahead of the diagonal neighbors.
        Pair<GridPosition, GridPosition> closestKey = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        final int kDistanceGridCells = (int) Math.ceil(cacheToleranceMeters / nodeSize);
        for (int i = -kDistanceGridCells; i <= kDistanceGridCells; ++i) {
            for (int j = -kDistanceGridCells; j <= kDistanceGridCells; ++j) {
                if (i == 0 && j == 0) continue;
                key =
                        new Pair<GridPosition, GridPosition>(
                                new GridPosition(start.x + i, start.y + j), goal);
                result = pathCache.getOrDefault(key, null);
                // System.out.println("[Cache] Checking cache at " + key.getFirst() + " -> " +
                // key.getSecond());
                if (result != null) {
                    // System.out.println("[Cache] Potential approx hit lookup " + key.getFirst() +
                    // " -> " +
                    // key.getSecond());
                    double hitDistance =
                            gridPosToTranslation2d(start)
                                    .getDistance(gridPosToTranslation2d(key.getFirst()));
                    if (hitDistance < closestDistance && hitDistance <= cacheToleranceMeters) {
                        closestDistance = hitDistance;
                        closestKey = key;
                    }
                }
            }
        }
        if (closestKey != null) {
            System.out.println(
                    "[Cache] Using approx hit " + key.getFirst() + " -> " + key.getSecond());
            return pathCache.getOrDefault(closestKey, null);
        }
        System.out.println("[Cache] Cache miss!");
        return null;
    }

    private void writeToCache(GridPosition start, GridPosition goal, PathPlannerPath path) {
        if (pathCache.size() < kMaxCacheSize) {
            System.out.println("[Cache] Write " + start + " -> " + goal);
            pathCache.put(new Pair<GridPosition, GridPosition>(start, goal), path.clone());
        }
    }

    private void doWork(
            boolean needsReset,
            boolean doMinor,
            GridPosition sStart,
            GridPosition sGoal,
            Pose2d realStartPose,
            Pose2d realGoalPose,
            Set<GridPosition> obstacles,
            PathConstraints constraints,
            Translation2d startVelocity) {
        if (needsReset) {
            reset(sStart, sGoal);
        }
        // We don't use dynamic obstacles, but if we did we'd want to check that too.
        if (enableCaching) {
            PathPlannerPath maybePath = checkCache(sStart, sGoal);
            if (maybePath != null) {
                PathPlannerPath path = maybePath;
                if (startVelocity.getNorm() > kMaxVelocityForBezierCache) {
                    // Recompute the path from the actual start state since the velocity is
                    // nonnegligible.
                    path = new PathPlannerPath(path, realStartPose, startVelocity);
                    System.out.println("[Cache] Regenerating bezier due to startVelocity");

                } else {
                    path = maybePath.clone(); // Deep copy so that subsequent path trimming doesn't
                    // change the cache.
                }

                pathLock.writeLock().lock();
                currentPath = path;
                pathLock.writeLock().unlock();
                newPathAvailable = true;
                return;
            }
        }
        if (doMinor) {
            computeOrImprovePath(sStart, sGoal, obstacles);

            List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles);
            List<Waypoint> waypoints =
                    createWaypoints(
                            pathPositions, realStartPose, realGoalPose, obstacles, startVelocity);
            PathPlannerPath path = null;
            if (waypoints.size() >= 2) {
                if (goalPath == null) {
                    path =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(0.0, realGoalPose.getRotation()));
                } else {
                    path =
                            PathPlannerPath.concatenatePaths(
                                    waypoints,
                                    new IdealStartingState(0.0, realStartPose.getRotation()),
                                    new GoalEndState(
                                            goalPath.getIdealStartingState().velocity(),
                                            realGoalPose.getRotation()),
                                    goalPath.getWaypoints(),
                                    goalPath.getIdealStartingState(),
                                    goalPath.getGoalEndState(),
                                    goalPath.getGlobalConstraints(),
                                    PathPlannerPath.ConcatenationStrategy.SMOOTH);
                }
            }

            if (enableCaching && startVelocity.getNorm() <= kMaxVelocityForBezierCache) {
                // Write to the cache
                writeToCache(sStart, sGoal, path);
            }

            pathLock.writeLock().lock();
            currentPath = path;
            currentPathFull = pathPositions;
            pathLock.writeLock().unlock();

            newPathAvailable = true;
        }
    }

    private List<GridPosition> extractPath(
            GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (sGoal.equals(sStart)) {
            return new ArrayList<>(Arrays.asList(sGoal));
        }

        List<GridPosition> path = new ArrayList<>();
        path.add(sStart);

        var s = sStart;

        for (int k = 0; k < 200; k++) {
            HashMap<GridPosition, Double> gList = new HashMap<>();

            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                gList.put(x, g.get(x));
            }

            Map.Entry<GridPosition, Double> min = Map.entry(sGoal, Double.POSITIVE_INFINITY);
            for (var entry : gList.entrySet()) {
                if (entry.getValue() < min.getValue()) {
                    min = entry;
                }
            }
            s = min.getKey();

            path.add(s);
            if (s.equals(sGoal)) {
                break;
            }
        }

        return path;
    }

    private List<GridPosition> shortcut(List<GridPosition> path, Set<GridPosition> obstacles) {
        // --- Input Validation and Edge Cases ---
        if (path == null || path.size() <= 2) {
            // Cannot shorten null, empty, single-point, or two-point paths.
            // Return a copy or an empty list.
            return path;
        }

        int n = path.size();
        // Stores the minimum distance from the start point path.get(0) to path.get(i)
        double[] minDist = new double[n];
        // Stores the index of the predecessor point in the optimal path to path.get(i)
        int[] predecessor = new int[n];

        // Initialize distances and predecessors
        Arrays.fill(minDist, Double.POSITIVE_INFINITY);
        Arrays.fill(predecessor, -1); // -1 indicates no predecessor / unreachable

        minDist[0] = 0.0; // Distance from start to start is 0

        // --- Dynamic Programming: Calculate Shortest Path in DAG ---
        for (int i = 0; i < n - 1; i++) {
            // If path.get(i) is reachable from the start
            if (Double.isFinite(minDist[i])) {
                // Check for possible direct paths (shortcuts) to all subsequent points path.get(j)
                for (int j = i + 1; j < n; j++) {
                    GridPosition currentPoint = path.get(i);
                    GridPosition targetPoint = path.get(j);

                    // Check if the direct line segment is collision-free
                    if ((j == i + 1) || walkable(currentPoint, targetPoint, obstacles)) {
                        double segmentDistance = heuristic(currentPoint, targetPoint);
                        double newDistToTarget = minDist[i] + segmentDistance;

                        // If this path to targetPoint is shorter than any found so far
                        if (newDistToTarget < minDist[j]) {
                            minDist[j] = newDistToTarget;
                            predecessor[j] = i; // Record 'i' as the best predecessor for 'j'
                        }
                    }
                }
            }
        }

        // --- Path Reconstruction ---
        List<GridPosition> shortPath =
                new LinkedList<>(); // Use LinkedList for efficient prepending
        int currentIdx = n - 1; // Start backtracking from the end point

        // Check if the end point was actually reachable
        if (!Double.isFinite(minDist[currentIdx])) {
            // The end node was unreachable. This implies an issue, possibly with
            // the original path or obstacle placement near the end.
            // Returning the original path is a safe fallback.
            System.err.println(
                    "Warning: End node of the path was unreachable during optimal shortcutting. Returning original path.");
            return path;
        }

        // Backtrack using predecessors until we reach the start point (index 0)
        while (currentIdx != -1) {
            shortPath.add(0, path.get(currentIdx)); // Prepend the current point
            currentIdx = predecessor[currentIdx]; // Move to the predecessor
        }

        return shortPath; // The list is already in the correct start-to-end order
    }

    private List<Waypoint> createWaypoints(
            List<GridPosition> path,
            Pose2d realStartPos,
            Pose2d realGoalPos,
            Set<GridPosition> obstacles,
            Translation2d startVelocity) {
        if (path.isEmpty()) {
            return new ArrayList<>();
        }

        List<GridPosition> simplifiedPath = shortcut(path, obstacles);
        if (simplifiedPath.size() == 1) simplifiedPath.add(simplifiedPath.get(0));

        List<Translation2d> fieldPosPath = new ArrayList<>();
        for (GridPosition pos : simplifiedPath) {
            fieldPosPath.add(gridPosToTranslation2d(pos));
        }

        if (fieldPosPath.size() < 2) {
            return new ArrayList<>();
        }

        // Replace start and end positions with their real positions
        fieldPosPath.set(0, realStartPos.getTranslation());
        fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos.getTranslation());

        List<Pose2d> pathPoses = new ArrayList<>();
        pathPoses.add(
                new Pose2d(
                        fieldPosPath.get(0),
                        fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));
        for (int i = 1; i < fieldPosPath.size() - 1; i++) {
            Translation2d last = fieldPosPath.get(i - 1);
            Translation2d current = fieldPosPath.get(i);
            Translation2d next = fieldPosPath.get(i + 1);

            Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
            Rotation2d heading1 = current.minus(last).getAngle();
            Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);
            Rotation2d heading2 = next.minus(anchor2).getAngle();
            if (i > 1) {
                pathPoses.add(new Pose2d(anchor1, heading1));
            }
            pathPoses.add(new Pose2d(anchor2, heading2));
        }
        pathPoses.add(
                new Pose2d(
                        fieldPosPath.get(fieldPosPath.size() - 1),
                        fieldPosPath
                                .get(fieldPosPath.size() - 1)
                                .minus(fieldPosPath.get(fieldPosPath.size() - 2))
                                .getAngle()));

        return PathPlannerPath.waypointsFromPoses(pathPoses, startVelocity);
    }

    private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
        if (!obstacles.contains(pos)) {
            return pos;
        }

        Set<GridPosition> visited = new HashSet<>();

        Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos));

        while (!queue.isEmpty()) {
            GridPosition check = queue.poll();
            if (!obstacles.contains(check)) {
                return check;
            }
            visited.add(check);

            for (GridPosition neighbor : getAllNeighbors(check)) {
                if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
                    queue.add(neighbor);
                }
            }
        }
        return null;
    }

    private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {
        int x0 = s1.x;
        int y0 = s1.y;
        int x1 = s2.x;
        int y1 = s2.y;

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int x = x0;
        int y = y0;
        int n = 1 + dx + dy;
        int xInc = (x1 > x0) ? 1 : -1;
        int yInc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; n--) {
            if (obstacles.contains(new GridPosition(x, y))) {
                return false;
            }

            if (error > 0) {
                x += xInc;
                error -= dy;
            } else if (error < 0) {
                y += yInc;
                error += dx;
            } else {
                x += xInc;
                y += yInc;
                error -= dy;
                error += dx;
                n--;
            }
        }

        return true;
    }

    private void reset(GridPosition sStart, GridPosition sGoal) {
        g.clear();
        rhs.clear();
        open.clear();
        incons.clear();
        closed.clear();

        for (int x = 0; x < nodesX; x++) {
            for (int y = 0; y < nodesY; y++) {
                g.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
                rhs.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
            }
        }

        rhs.put(sGoal, 0.0);

        eps = EPS;

        open.put(sGoal, key(sGoal, sStart));
    }

    private void computeOrImprovePath(
            GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        while (true) {
            var sv = topKey();
            if (sv == null) {
                break;
            }
            var s = sv.getFirst();
            var v = sv.getSecond();

            if (comparePair(v, key(sStart, sStart)) >= 0 && rhs.get(sStart).equals(g.get(sStart))) {
                break;
            }

            open.remove(s);

            if (g.get(s) > rhs.get(s)) {
                g.put(s, rhs.get(s));
                closed.add(s);

                for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
                    updateState(sn, sStart, sGoal, obstacles);
                }
            } else {
                g.put(s, Double.POSITIVE_INFINITY);
                for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
                    updateState(sn, sStart, sGoal, obstacles);
                }
                updateState(s, sStart, sGoal, obstacles);
            }
        }
    }

    private void updateState(
            GridPosition s, GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (!s.equals(sGoal)) {
            rhs.put(s, Double.POSITIVE_INFINITY);

            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                rhs.put(s, Math.min(rhs.get(s), g.get(x) + cost(s, x, obstacles)));
            }
        }

        open.remove(s);

        if (!g.get(s).equals(rhs.get(s))) {
            if (!closed.contains(s)) {
                open.put(s, key(s, sStart));
            } else {
                incons.put(s, Pair.of(0.0, 0.0));
            }
        }
    }

    private double cost(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (isCollision(sStart, sGoal, obstacles)) {
            return Double.POSITIVE_INFINITY;
        }

        return heuristic(sStart, sGoal);
    }

    private boolean isCollision(
            GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
        if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
            return true;
        }

        if (sStart.x != sEnd.x && sStart.y != sEnd.y) {
            GridPosition s1;
            GridPosition s2;

            if (sEnd.x - sStart.x == sStart.y - sEnd.y) {
                s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
                s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
            } else {
                s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
                s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
            }

            return obstacles.contains(s1) || obstacles.contains(s2);
        }

        return false;
    }

    private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
        List<GridPosition> ret = new ArrayList<>();

        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
                if (!obstacles.contains(sNext)
                        && sNext.x >= 0
                        && sNext.x < nodesX
                        && sNext.y >= 0
                        && sNext.y < nodesY) {
                    ret.add(sNext);
                }
            }
        }
        return ret;
    }

    private List<GridPosition> getAllNeighbors(GridPosition s) {
        List<GridPosition> ret = new ArrayList<>();

        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
                if (sNext.x >= 0 && sNext.x < nodesX && sNext.y >= 0 && sNext.y < nodesY) {
                    ret.add(sNext);
                }
            }
        }
        return ret;
    }

    private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
        if (g.get(s) > rhs.get(s)) {
            return Pair.of(rhs.get(s) + eps * heuristic(sStart, s), rhs.get(s));
        } else {
            return Pair.of(g.get(s) + heuristic(sStart, s), g.get(s));
        }
    }

    private Pair<GridPosition, Pair<Double, Double>> topKey() {
        Map.Entry<GridPosition, Pair<Double, Double>> min = null;
        for (var entry : open.entrySet()) {
            if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) {
                min = entry;
            }
        }

        if (min == null) {
            return null;
        }

        return Pair.of(min.getKey(), min.getValue());
    }

    private double heuristic(GridPosition sStart, GridPosition sGoal) {
        return Math.hypot(sGoal.x - sStart.x, sGoal.y - sStart.y);
    }

    private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
        int first = Double.compare(a.getFirst(), b.getFirst());
        if (first == 0) {
            return Double.compare(a.getSecond(), b.getSecond());
        } else {
            return first;
        }
    }

    private GridPosition getGridPos(Translation2d pos) {
        int x = (int) Math.floor(pos.getX() / nodeSize);
        int y = (int) Math.floor(pos.getY() / nodeSize);

        return new GridPosition(x, y);
    }

    private Translation2d gridPosToTranslation2d(GridPosition pos) {
        return new Translation2d(
                (pos.x * nodeSize) + (nodeSize / 2.0), (pos.y * nodeSize) + (nodeSize / 2.0));
    }

    /**
     * Represents a node in the pathfinding grid
     *
     * @param x X index in the grid
     * @param y Y index in the grid
     */
    public record GridPosition(int x, int y) implements Comparable<GridPosition> {
        @Override
        public int compareTo(GridPosition o) {
            if (x == o.x) {
                return Integer.compare(y, o.y);
            } else {
                return Integer.compare(x, o.x);
            }
        }
    }
}
