package com.team254.lib.pathplanner.pathfinding;

import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Static class for interacting with the chosen pathfinding implementation from the pathfinding
 * commands
 */
public class Pathfinding {
    private static Pathfinder pathfinder = null;

    /**
     * Set the pathfinder that should be used by the path following commands
     *
     * @param pathfinder The pathfinder to use
     */
    public static void setPathfinder(Pathfinder pathfinder) {
        Pathfinding.pathfinder = pathfinder;
    }

    /** Ensure that a pathfinding implementation has been chosen. If not, set it to the default. */
    public static void ensureInitialized() {
        if (pathfinder == null) {
            // Hasn't been initialized yet, use the default implementation

            pathfinder = new LocalADStar();
        }
    }

    /**
     * Get if a new path has been calculated since the last time a path was retrieved
     *
     * @return True if a new path is available
     */
    public static boolean isNewPathAvailable() {
        return pathfinder.isNewPathAvailable();
    }

    /**
     * Get the most recently calculated path
     *
     * @return The PathPlannerPath created from the points calculated by the pathfinder
     */
    public static PathPlannerPath getCurrentPath() {
        return pathfinder.getCurrentPath();
    }

    /**
     * Set the start position to pathfind from
     *
     * @param startPosition Start position on the field. If this is within an obstacle it will be
     *     moved to the nearest non-obstacle node.
     */
    public static void setStartPose(Pose2d startPose) {
        pathfinder.setStartPose(startPose);
    }

    /**
     * Set the goal position to pathfind to
     *
     * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
     *     to the nearest non-obstacle node.
     */
    public static void setGoalPose(Pose2d goalPose) {
        pathfinder.setGoalPose(goalPose);
    }

    public static void setStartVelocity(Translation2d startVelocity) {
        pathfinder.setStartVelocity(startVelocity);
    }

    public static void setConstraints(PathConstraints constraints) {
        pathfinder.setConstraints(constraints);
    }

    public static void setProblem(
            Pose2d startPose,
            Pose2d goalPose,
            Translation2d startVelocity,
            PathConstraints constraints) {
        pathfinder.setProblem(startPose, goalPose, startVelocity, constraints);
    }

    public static void setProblem(
            Pose2d startPose,
            PathPlannerPath goalPath,
            Translation2d startVelocity,
            PathConstraints constraints) {
        pathfinder.setProblem(startPose, goalPath, startVelocity, constraints);
    }

    public static void setTeleopObstacles() {
        pathfinder.setTeleopObstacles();
    }

    public static void setAutoObstacles() {
        pathfinder.setAutoObstacles();
    }

    public static void setBackoffObstacles() {
        pathfinder.setBackoffObstacles();
    }

    public static void setCacheDistanceToleranceMeters(double tolerance) {
        pathfinder.setCacheDistanceToleranceMeters(tolerance);
    }

    public static void enableCaching() {
        pathfinder.enableCaching();
    }

    public static void disableCaching() {
        pathfinder.disableCaching();
    }

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d
     *     represents opposite corners of a bounding box.
     * @param currentRobotPos The current position of the robot. This is needed to change the start
     *     position of the path if the robot is now within an obstacle.
     */
    public static void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs,
            Pose2d currentRobotPose,
            Translation2d startVelocity) {
        pathfinder.setDynamicObstacles(obs, currentRobotPose, startVelocity);
    }
}
