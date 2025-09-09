package com.team254.lib.pathplanner.pathfinding;

import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** Interface for a pathfinder that can be used by PPLib's pathfinding commands */
public interface Pathfinder {
    /**
     * Get if a new path has been calculated since the last time a path was retrieved
     *
     * @return True if a new path is available
     */
    boolean isNewPathAvailable();

    /**
     * Get the most recently calculated path
     *
     * @return The PathPlannerPath created from the points calculated by the pathfinder
     */
    PathPlannerPath getCurrentPath();

    /**
     * Set the start position to pathfind from
     *
     * @param startPosition Start position on the field. If this is within an obstacle it will be
     *     moved to the nearest non-obstacle node.
     */
    void setStartPose(Pose2d startPose);

    /**
     * Set the goal position to pathfind to
     *
     * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
     *     to the nearest non-obstacle node.
     */
    void setGoalPose(Pose2d goalPose);

    void setStartVelocity(Translation2d startVelocity);

    void setConstraints(PathConstraints constraints);

    void setTeleopObstacles();

    void setAutoObstacles();

    void setBackoffObstacles();

    // Convenience method for all of the above atomically.
    void setProblem(
            Pose2d startPose,
            Pose2d goalPose,
            Translation2d startVelocity,
            PathConstraints constraints);

    void setProblem(
            Pose2d startPose,
            PathPlannerPath goalPath,
            Translation2d startVelocity,
            PathConstraints constraints);

    void enableCaching();

    void disableCaching();

    void setCacheDistanceToleranceMeters(double tolerance);

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d
     *     represents opposite corners of a bounding box.
     * @param currentRobotPos The current position of the robot. This is needed to change the start
     *     position of the path to properly avoid obstacles
     */
    void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs,
            Pose2d currentRobotPose,
            Translation2d startVelocity);
}
