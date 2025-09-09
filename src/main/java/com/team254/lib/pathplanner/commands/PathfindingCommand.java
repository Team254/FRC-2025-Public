package com.team254.lib.pathplanner.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team254.lib.pathplanner.config.ModuleConfig;
import com.team254.lib.pathplanner.config.RobotConfig;
import com.team254.lib.pathplanner.events.EventScheduler;
import com.team254.lib.pathplanner.path.GoalEndState;
import com.team254.lib.pathplanner.path.IPathCallback;
import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import com.team254.lib.pathplanner.path.PathPoint;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import com.team254.lib.pathplanner.util.PathPlannerLogging;
import com.team254.lib.util.MathHelpers;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Base pathfinding command */
public class PathfindingCommand extends Command {
    private static int instances = 0;

    private final Timer timer = new Timer();
    private final PathPlannerPath targetPath;
    private Pose2d targetPose;
    private GoalEndState goalEndState;
    private final PathConstraints constraints;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final RobotConfig robotConfig;

    private PathPlannerPath currentPath;
    private PathPlannerTrajectory currentTrajectory;
    private EventScheduler eventScheduler = new EventScheduler();

    private Consumer<PathPlannerTrajectory> controller;

    private IPathCallback pathCallback;

    private boolean beginGenerateJoinPath = false;
    private boolean skipUpdates = false;

    /**
     * Constructs a new base pathfinding command that will generate a path towards the given path.
     *
     * @param targetPath the path to pathfind to
     * @param constraints the path constraints to use while pathfinding
     * @param poseSupplier a supplier for the robot's current pose
     * @param speedsSupplier a supplier for the robot's current robot relative speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Should the target path be flipped to the other side of the field? This
     *     will maintain a global blue alliance origin.
     * @param requirements the subsystems required by this command
     */
    public PathfindingCommand(
            PathPlannerPath targetPath,
            PathConstraints constraints,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Subsystem... requirements) {
        this(
                targetPath,
                null,
                constraints,
                poseSupplier,
                speedsSupplier,
                controller,
                robotConfig,
                shouldFlipPath,
                requirements);
    }

    public PathfindingCommand(
            PathPlannerPath targetPath,
            IPathCallback pathCallback,
            PathConstraints constraints,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Subsystem... requirements) {
        addRequirements(requirements);

        Pathfinding.ensureInitialized();

        Rotation2d targetRotation = Rotation2d.kZero;
        double goalEndVel = targetPath.getIdealStartingState().velocityMPS();
        if (targetPath.isChoreoPath()) {
            // Can get() here without issue since all choreo trajectories have ideal
            // trajectories
            PathPlannerTrajectory choreoTraj =
                    targetPath.getIdealTrajectory(robotConfig).orElseThrow();
            targetRotation = choreoTraj.getInitialState().pose.getRotation();
            goalEndVel = choreoTraj.getInitialState().linearVelocity;
        } else {
            for (PathPoint p : targetPath.getAllPathPoints()) {
                if (p.rotationTarget != null) {
                    targetRotation = p.rotationTarget.rotation();
                    break;
                }
            }
        }

        this.targetPath = targetPath;
        this.pathCallback = pathCallback;
        this.targetPose = new Pose2d(this.targetPath.getPoint(0).position, targetRotation);
        this.goalEndState = new GoalEndState(goalEndVel, targetRotation);
        this.constraints = constraints;
        this.controller = controller;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.robotConfig = robotConfig;

        instances++;
        HAL.report(tResourceType.kResourceType_PathFindingCommand, instances);
    }

    /**
     * Constructs a new base pathfinding command that will generate a path towards the given pose.
     *
     * @param targetPose the pose to pathfind to, the rotation component is only relevant for
     *     holonomic drive trains
     * @param constraints the path constraints to use while pathfinding
     * @param goalEndVel The goal end velocity when reaching the target pose
     * @param poseSupplier a supplier for the robot's current pose
     * @param speedsSupplier a supplier for the robot's current robot relative speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param requirements the subsystems required by this command
     */
    public PathfindingCommand(
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVel,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            Subsystem... requirements) {
        addRequirements(requirements);

        Pathfinding.ensureInitialized();

        this.targetPath = null;
        this.targetPose = targetPose;
        this.goalEndState = new GoalEndState(goalEndVel, targetPose.getRotation());
        this.constraints = constraints;
        this.controller = controller;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.robotConfig = robotConfig;

        instances++;
        HAL.report(tResourceType.kResourceType_PathFindingCommand, instances);
    }

    /**
     * Constructs a new base pathfinding command that will generate a path towards the given pose.
     *
     * @param targetPose the pose to pathfind to, the rotation component is only relevant for
     *     holonomic drive trains
     * @param constraints the path constraints to use while pathfinding
     * @param goalEndVel The goal end velocity when reaching the target pose
     * @param poseSupplier a supplier for the robot's current pose
     * @param speedsSupplier a supplier for the robot's current robot relative speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param requirements the subsystems required by this command
     */
    public PathfindingCommand(
            Pose2d targetPose,
            PathConstraints constraints,
            LinearVelocity goalEndVel,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            Subsystem... requirements) {
        this(
                targetPose,
                constraints,
                goalEndVel.in(MetersPerSecond),
                poseSupplier,
                speedsSupplier,
                controller,
                robotConfig,
                requirements);
    }

    /**
     * Constructs a new base pathfinding command that will generate a path towards the given pose.
     *
     * @param targetPose the pose to pathfind to, the rotation component is only relevant for
     *     holonomic drive trains
     * @param constraints the path constraints to use while pathfinding
     * @param poseSupplier a supplier for the robot's current pose
     * @param speedsSupplier a supplier for the robot's current robot relative speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param requirements the subsystems required by this command
     */
    public PathfindingCommand(
            Pose2d targetPose,
            PathConstraints constraints,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            Subsystem... requirements) {
        this(
                targetPose,
                constraints,
                0.0,
                poseSupplier,
                speedsSupplier,
                controller,
                robotConfig,
                requirements);
    }

    @Override
    public void initialize() {
        currentTrajectory = null;
        Pose2d currentPose = poseSupplier.get();

        if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) <= 0.0) {
            controller.accept(PathPlannerTrajectory.makeStayInPlaceTrajectory());
        } else {
            var currentSpeeds = speedsSupplier.get();
            ChassisSpeeds fieldRelativeCurrentSpeeds =
                    ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());

            // Expect it takes at least one command cycle plus up to one loop of the Pathfinding
            // thread to
            // get a new path.
            final double kLoopTime = 0.03;
            Pose2d predictedPose =
                    currentPose.exp(
                            new Twist2d(
                                    kLoopTime * currentSpeeds.vxMetersPerSecond,
                                    kLoopTime * currentSpeeds.vyMetersPerSecond,
                                    kLoopTime * currentSpeeds.omegaRadiansPerSecond));

            if (targetPath == null) {
                Pathfinding.setProblem(
                        predictedPose,
                        targetPose,
                        new Translation2d(
                                fieldRelativeCurrentSpeeds.vxMetersPerSecond,
                                fieldRelativeCurrentSpeeds.vyMetersPerSecond),
                        constraints);
            } else {
                Pathfinding.setProblem(
                        predictedPose,
                        targetPath,
                        new Translation2d(
                                fieldRelativeCurrentSpeeds.vxMetersPerSecond,
                                fieldRelativeCurrentSpeeds.vyMetersPerSecond),
                        constraints);
            }
            Logger.recordOutput("PathPlanner/Finish", false);
            Logger.recordOutput("PathPlanner/GoalPosition", targetPose.getTranslation());
        }
        Logger.recordOutput("PathPlanner/currentPathSize", -1);
    }

    @Override
    public void execute() {
        Logger.recordOutput("PathPlanner/RemainingTime", getRemainingTime());

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        PathPlannerLogging.logCurrentPose(currentPose);

        // Skip new paths if we are close to the end
        skipUpdates =
                skipUpdates
                        || (currentTrajectory != null
                                && currentPose
                                                .getTranslation()
                                                .getDistance(targetPose.getTranslation())
                                        < 0.5);
        Logger.recordOutput("PathPlanner/newPathAvailable", Pathfinding.isNewPathAvailable());
        boolean pathUpdated = false;
        if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
            currentPath = Pathfinding.getCurrentPath();

            if (currentPath != null) {
                Logger.recordOutput("PathPlanner/currentPathSize", currentPath.numPoints());
                pathUpdated = true;
            }
        }
        if (currentPath != null
                && (pathUpdated || (pathCallback != null && pathCallback.markupUpdated()))) {
            if (pathCallback != null) {
                currentPath.applyCallback(pathCallback);
            }
            List<Pose2d> anchors = new ArrayList<Pose2d>();
            for (var waypoint : currentPath.getWaypoints()) {
                anchors.add(
                        new Pose2d(
                                waypoint.anchor(),
                                waypoint.nextControl() == null
                                        ? waypoint.anchor().minus(waypoint.prevControl()).getAngle()
                                        : waypoint.nextControl()
                                                .minus(waypoint.anchor())
                                                .getAngle()));
            }
            Logger.recordOutput("PathPlanner/anchors", anchors.toArray(new Pose2d[0]));

            // Fast-forward path to current position in the case the path is geometrically the same
            // but
            // constraints have changed.
            Optional<PathPlannerTrajectoryState> preTrimSetpoint = Optional.empty();
            if (!pathUpdated && currentTrajectory != null) {
                // Fast-forward path to current position by looking up the waypointRelativePos of
                // the
                // current trajectory sample.
                double t = timer.get();
                preTrimSetpoint = Optional.of(currentTrajectory.sample(t));
                currentPath.trimStart(preTrimSetpoint.get().waypointRelativePos);
            } else if (pathUpdated) {
                // Fast-forward path to current position in the case the path has actually changed
                // geometry.
                // Find the closest state. Do not terminate early, because paths can have multiple
                // local
                // approaches
                // to the current robot pose.
                int closestStateIdx = 0;
                var allPoints = currentPath.getAllPathPoints();
                double closestDistance =
                        allPoints.get(0).position.getDistance(currentPose.getTranslation());
                Translation2d prevPoint = allPoints.get(0).position;

                Optional<Rotation2d> curHeading = Optional.empty();
                if (Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                        > .1) {
                    curHeading =
                            Optional.of(
                                    new Rotation2d(
                                            currentSpeeds.vxMetersPerSecond,
                                            currentSpeeds.vyMetersPerSecond));
                }
                double[] distances = new double[allPoints.size()];
                distances[0] = closestDistance;
                for (int i = 1; i < allPoints.size() - 1; ++i) {
                    Translation2d point = allPoints.get(i).position;
                    distances[i] = point.getDistance(currentPose.getTranslation());

                    Rotation2d heading = point.minus(prevPoint).getAngle();
                    prevPoint = point;
                    double headingDiffRads =
                            curHeading.isEmpty()
                                    ? 0.0
                                    : Math.abs(heading.minus(curHeading.get()).getRadians());
                    if (distances[i] < closestDistance && headingDiffRads < Math.PI * 0.5) {
                        closestDistance = distances[i];
                        closestStateIdx = i;
                    }
                }

                // Now figure out if we should interp between the state ahead or the state behind.
                double behindDist =
                        (closestStateIdx <= 0 ? Double.MAX_VALUE : distances[closestStateIdx - 1]);
                double aheadDist =
                        (closestStateIdx >= allPoints.size() - 1
                                ? Double.MAX_VALUE
                                : distances[closestStateIdx + 1]);
                // Find the two closest states in front of and behind robot
                int closestState1Idx = closestStateIdx - 1;
                int closestState2Idx = closestStateIdx;
                if (aheadDist < behindDist) {
                    ++closestState1Idx;
                    ++closestState2Idx;
                }
                if (closestState2Idx < allPoints.size() && closestState1Idx >= 0) {
                    // Use the closest 2 states to interpolate what the time offset should be
                    // This will account for the delay in pathfinding
                    var closestState1 = allPoints.get(closestState1Idx);
                    var closestState2 = allPoints.get(closestState2Idx);

                    double t =
                            MathHelpers.reverseInterpolate(
                                    currentPose.getTranslation(),
                                    closestState1.position,
                                    closestState2.position);
                    double waypointRelativePosShift =
                            MathUtil.interpolate(
                                    closestState1.waypointRelativePos,
                                    closestState2.waypointRelativePos,
                                    t);
                    currentPath.trimStart(waypointRelativePosShift);
                }
            }
            if (currentPath.getAllPathPoints().size() > 0) {

                // Cancel any commands that were already scheduled on the prior trajectory.
                eventScheduler.end();
                // Reset the timer now, to ensure we account for trajectory generation time.
                timer.reset();
                timer.start();
                currentPose = poseSupplier.get();
                if (preTrimSetpoint.isPresent()) {
                    // Initialize the trimmed trajectory from the last setpoint for continuity.
                    currentTrajectory =
                            new PathPlannerTrajectory(
                                    currentPath,
                                    preTrimSetpoint.get().fieldSpeeds,
                                    preTrimSetpoint.get().pose.getRotation(),
                                    robotConfig);
                } else {
                    // Initialize from current pose.
                    currentTrajectory =
                            new PathPlannerTrajectory(
                                    currentPath,
                                    currentSpeeds,
                                    currentPose.getRotation(),
                                    robotConfig);
                }

                controller.accept(currentTrajectory);
                eventScheduler.initialize(currentTrajectory);

                PathPlannerLogging.logActivePath(currentPath);
            }
        }

        if (currentTrajectory != null) {
            // ACTUAL following now happens in the controller thread, but we can still traverse the
            // trajectory in this thread to fire off
            // events and track progress.
            double t = timer.get();
            var targetState = currentTrajectory.sample(t);

            PathPlannerLogging.logCurrentPose(currentPose);
            PathPlannerLogging.logTargetPose(targetState.pose);

            Logger.recordOutput(
                    "PathPlanner/feedforwardXForces",
                    targetState.feedforwards.robotRelativeForcesXNewtons());
            Logger.recordOutput(
                    "PathPlanner/feedforwardYForces",
                    targetState.feedforwards.robotRelativeForcesYNewtons());

            eventScheduler.execute(t);
        }
    }

    @Override
    public boolean isFinished() {
        if (currentTrajectory != null) {
            if (timer.hasElapsed(currentTrajectory.getTotalTimeSeconds())) {
                // Also sanity check the actual pose.
                var pose = poseSupplier.get();
                var setpoint = currentTrajectory.getEndState().pose;
                final double kTolerance = 0.2;
                final double kFinishTime = 0.25;
                if (pose.getTranslation().getDistance(setpoint.getTranslation()) > kTolerance) {
                    return timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() + kFinishTime);
                } else {
                    return true;
                }
            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("PathPlanner/InterruptedEnd", interrupted);
        timer.stop();
        eventScheduler.end();

        // Only output 0 speeds when ending a path that is supposed to stop, this allows
        // interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!interrupted && goalEndState.velocityMPS() < 0.1 || DriverStation.isDisabled()) {
            controller.accept(PathPlannerTrajectory.makeStayInPlaceTrajectory());
        } else {
            controller.accept(null);
        }

        PathPlannerLogging.logActivePath(null);
    }

    public double getRemainingTime() {
        if (currentTrajectory == null) {
            return Double.POSITIVE_INFINITY;
        }
        return currentTrajectory.getTotalTimeSeconds() - (timer.get());
    }

    public PathPlannerTrajectoryState getPredictedSample(double time) {
        if (currentTrajectory == null) {
            return null;
        }
        return currentTrajectory.sample(timer.get() + time);
    }

    public boolean getBeginGenerateJoinPath() {
        return beginGenerateJoinPath;
    }

    public boolean getTrajectoryIsNull() {
        return currentTrajectory == null;
    }

    /**
     * Create a command to warmup the pathfinder and pathfinding command
     *
     * @return Pathfinding warmup command
     */
    public static Command warmupCommand() {
        return new PathfindingCommand(
                        new Pose2d(1.0, 4.0, Rotation2d.k180deg),
                        new PathConstraints(10, 10, 5, 5, 10, 10),
                        () -> new Pose2d(1.5, 4, Rotation2d.kZero),
                        ChassisSpeeds::new,
                        (trajectory) -> {},
                        new RobotConfig(
                                75,
                                6.8,
                                new ModuleConfig(
                                        0.048,
                                        10.0,
                                        1.2,
                                        DCMotor.getKrakenX60(1).withReduction(6.14),
                                        60.0,
                                        1),
                                0.55))
                .andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
                .ignoringDisable(true);
    }
}
