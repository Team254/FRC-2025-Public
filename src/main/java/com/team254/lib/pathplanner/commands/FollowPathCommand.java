package com.team254.lib.pathplanner.commands;

import com.team254.lib.pathplanner.config.ModuleConfig;
import com.team254.lib.pathplanner.config.RobotConfig;
import com.team254.lib.pathplanner.events.EventScheduler;
import com.team254.lib.pathplanner.path.*;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team254.lib.pathplanner.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Base command for following a path */
public class FollowPathCommand extends Command {
    private final Timer timer = new Timer();
    private final PathPlannerPath originalPath;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Consumer<PathPlannerTrajectory> controller;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;
    private final EventScheduler eventScheduler;

    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;

    /**
     * Construct a base path following command
     *
     * @param path The path to follow
     * @param poseSupplier Function that supplies the current field-relative pose of the robot
     * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
     *     maintain a global blue alliance origin.
     * @param requirements Subsystems required by this command, usually just the drive subsystem
     */
    public FollowPathCommand(
            PathPlannerPath path,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Subsystem... requirements) {
        this.originalPath = path;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.controller = controller;
        this.robotConfig = robotConfig;
        this.shouldFlipPath = shouldFlipPath;
        this.eventScheduler = new EventScheduler();

        Set<Subsystem> driveRequirements = Set.of(requirements);
        addRequirements(requirements);

        // Add all event scheduler requirements to this command's requirements
        var eventReqs = EventScheduler.getSchedulerRequirements(this.originalPath);
        if (!Collections.disjoint(driveRequirements, eventReqs)) {
            throw new IllegalArgumentException(
                    "Events that are triggered during path following cannot require the drive subsystem");
        }
        addRequirements(eventReqs);

        this.path = this.originalPath;
        // Ensure the ideal trajectory is generated
        Optional<PathPlannerTrajectory> idealTrajectory =
                this.path.getIdealTrajectory(this.robotConfig);
        idealTrajectory.ifPresent(traj -> this.trajectory = traj);
    }

    @Override
    public void initialize() {
        if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
            path = originalPath.flipPath();
        } else {
            path = originalPath;
        }

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        // controller.reset(currentPose, currentSpeeds);

        double linearVel =
                Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        if (path.getIdealStartingState() != null) {
            // Check if we match the ideal starting state
            boolean idealVelocity =
                    Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
            boolean idealRotation =
                    !robotConfig.isHolonomic
                            || Math.abs(
                                            currentPose
                                                    .getRotation()
                                                    .minus(path.getIdealStartingState().rotation())
                                                    .getDegrees())
                                    <= 30.0;
            if (idealVelocity && idealRotation) {
                // We can use the ideal trajectory
                trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
            } else {
                // We need to regenerate
                trajectory =
                        path.generateTrajectory(
                                currentSpeeds, currentPose.getRotation(), robotConfig);
            }
        } else {
            // No ideal starting state, generate the trajectory
            trajectory =
                    path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
        }

        controller.accept(trajectory);
        PathPlannerAuto.setCurrentTrajectory(trajectory);
        PathPlannerAuto.currentPathName = originalPath.name;

        PathPlannerLogging.logActivePath(path);

        eventScheduler.initialize(trajectory);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        var targetState = trajectory.sample(currentTime);
        Logger.recordOutput("PathPlanner/TargetStateFieldSpeeds", targetState.fieldSpeeds);
        Logger.recordOutput(
                "PathPlanner/TargetStateFinalPointPos",
                path.getPoint(path.getAllPathPoints().size() - 1).rotationTarget);
        Logger.recordOutput("PathPlanner/RemainingTime", getRemainingTime());

        Pose2d currentPose = poseSupplier.get();

        PathPlannerLogging.logCurrentPose(currentPose);
        PathPlannerLogging.logTargetPose(targetState.pose);

        eventScheduler.execute(currentTime);
    }

    public double getRemainingTime() {
        if (trajectory == null) {
            return Double.POSITIVE_INFINITY;
        }
        return trajectory.getTotalTimeSeconds() - timer.get();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        PathPlannerAuto.currentPathName = "";
        PathPlannerAuto.setCurrentTrajectory(null);

        // Only output 0 speeds when ending a path that is supposed to stop, this allows
        // interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1
                || DriverStation.isDisabled()) {
            controller.accept(PathPlannerTrajectory.makeStayInPlaceTrajectory());
        } else {
            controller.accept(null);
        }

        PathPlannerLogging.logActivePath(null);

        eventScheduler.end();
    }

    /**
     * Create a command to warmup on-the-fly generation, replanning, and the path following command
     *
     * @return Path following warmup command
     */
    public static Command warmupCommand() {
        List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(0.0, 0.0, Rotation2d.kZero),
                        new Pose2d(6.0, 6.0, Rotation2d.kZero));
        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        new PathConstraints(4.0, 4.0, 4.0, 4.0, 4.0, 4.0),
                        new IdealStartingState(0.0, Rotation2d.kZero),
                        new GoalEndState(0.0, Rotation2d.kCW_90deg));

        return new FollowPathCommand(
                        path,
                        () -> Pose2d.kZero,
                        ChassisSpeeds::new,
                        (trajectory) -> {},
                        new RobotConfig(
                                75,
                                6.8,
                                new ModuleConfig(
                                        0.048,
                                        5.0,
                                        1.2,
                                        DCMotor.getKrakenX60(1).withReduction(6.14),
                                        60.0,
                                        1),
                                0.55),
                        () -> true)
                .andThen(Commands.print("[PathPlanner] FollowPathCommand finished warmup"))
                .ignoringDisable(true);
    }
}
