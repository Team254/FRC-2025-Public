package com.team254.frc2025.commands;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.drive.DriveSubsystem;
import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.path.ConstraintsZone;
import com.team254.lib.pathplanner.path.EventMarker;
import com.team254.lib.pathplanner.path.GoalEndState;
import com.team254.lib.pathplanner.path.IPathCallback;
import com.team254.lib.pathplanner.path.IdealStartingState;
import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.pathplanner.util.FlippingUtil;
import com.team254.lib.reefscape.ReefBranch;
import com.team254.lib.reefscape.ScoringLocation;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.MathHelpers;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathfindingAutoAlignCommand extends Command {
    private final RobotState robotState;
    private final Supplier<ReefBranch> branchSupplier;
    private final Supplier<Boolean> letterSupplier;
    private final Supplier<Double> heightSupplier;
    private final Supplier<Command> stageCommandSupplier;
    private final BooleanSupplier doneStagingSupplier;
    private final BooleanSupplier isRedAlliance;
    private final boolean isAlgae;
    private final boolean reefBackoff;
    private final boolean shortJoinPath;

    private PathPlannerPath joinPath = null;

    static final double kPathfindSpeedScalingFactor = 0.8;
    static final double kPathfindAccelScalingFactor = 0.8;
    static final double kPathfindYAccelScalingFactor = 0.4;
    static final double kPathfindAngularScalingFactor = 0.8;
    static final PathConstraints pathfindConstraints =
            new PathConstraints(
                    Constants.DriveConstants.kDriveMaxSpeed * kPathfindSpeedScalingFactor,
                    Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                            * kPathfindAccelScalingFactor,
                    Constants.DriveConstants.kDriveMaxAngularRate * kPathfindAngularScalingFactor,
                    Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            * kPathfindAngularScalingFactor,
                    Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                            * kPathfindAccelScalingFactor,
                    Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                            * kPathfindYAccelScalingFactor);

    static final double kSlowPathfindSpeedScalingFactor = 0.8;
    static final double kSlowPathfindAccelScalingFactor = 0.6;
    static final double kSlowPathfindAccelXScalingFactor = 0.45;
    static final double kSlowPathfindAccelYScalingFactor = 0.35;
    static final double kSlowPathfindAngularScalingFactor = 0.8;
    static final PathConstraints slowPathfindConstraints =
            new PathConstraints(
                    Constants.DriveConstants.kDriveMaxSpeed * kSlowPathfindSpeedScalingFactor,
                    Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                            * kSlowPathfindAccelScalingFactor,
                    Constants.DriveConstants.kDriveMaxAngularRate
                            * kSlowPathfindAngularScalingFactor,
                    Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            * kSlowPathfindAngularScalingFactor,
                    Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                            * kSlowPathfindAccelXScalingFactor,
                    Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                            * kSlowPathfindAccelYScalingFactor);

    static final double kPathfindToFeederSpeedScalingFactor = 0.825;
    static final double kPathfindToFeederAccelScalingFactor = 1.15;
    static final PathConstraints pathfindToFeederConstraints =
            new PathConstraints(
                    Constants.DriveConstants.kDriveMaxSpeed * kPathfindToFeederSpeedScalingFactor,
                    Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                            * kPathfindToFeederAccelScalingFactor,
                    Constants.DriveConstants.kDriveMaxAngularRate
                            * kPathfindToFeederSpeedScalingFactor,
                    Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            * kPathfindToFeederAccelScalingFactor,
                    Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                            * kPathfindToFeederAccelScalingFactor,
                    Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                            * kPathfindToFeederAccelScalingFactor);

    static final double kJoinPathSpeedScalingFactor = 0.6;
    static final double kJoinPathAccelSpeedScalingFactor = 0.5;
    static final double kJoinPathAngularScalingFactor = 0.8;
    static final PathConstraints joinPathConstraints =
            new PathConstraints(
                    Math.min(
                            Constants.AutoConstants.kMaxEndPathVelocity,
                            Constants.DriveConstants.kDriveMaxSpeed * kJoinPathSpeedScalingFactor),
                    Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                            * kJoinPathAccelSpeedScalingFactor,
                    Constants.DriveConstants.kDriveMaxAngularRate * kJoinPathAngularScalingFactor,
                    Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            * kJoinPathAngularScalingFactor,
                    Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                            * kJoinPathAccelSpeedScalingFactor,
                    Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                            * kJoinPathAccelSpeedScalingFactor);

    static final double kSlowJoinPathSpeedScalingFactor = 0.6;
    static final double kSlowJoinPathAccelScalingFactor = 0.3;
    static final double kSlowJoinPathAccelYScalingFactor = 0.25;
    static final double kSlowJoinPathAngularScalingFactor = 0.8;
    static final PathConstraints slowJoinPathConstraints =
            new PathConstraints(
                    Math.min(
                            Constants.AutoConstants.kMaxEndPathVelocity,
                            Constants.DriveConstants.kDriveMaxSpeed
                                    * kSlowJoinPathSpeedScalingFactor),
                    Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                            * kSlowJoinPathAccelScalingFactor,
                    Constants.DriveConstants.kDriveMaxAngularRate
                            * kSlowJoinPathAngularScalingFactor,
                    Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            * kSlowJoinPathAngularScalingFactor,
                    Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                            * kSlowJoinPathAccelScalingFactor,
                    Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                            * kSlowJoinPathAccelYScalingFactor);

    private Command innerCommand;

    public PathfindingAutoAlignCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            Supplier<Double> heightSupplier,
            Supplier<Command> stageCommandSupplier,
            BooleanSupplier doneStagingSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae) {
        this.robotState = robotState;
        this.branchSupplier = branchSupplier;
        this.letterSupplier = letterSupplier;
        this.heightSupplier = heightSupplier;
        this.stageCommandSupplier = stageCommandSupplier;
        this.doneStagingSupplier = doneStagingSupplier;
        this.isRedAlliance = isRedAlliance;
        this.isAlgae = isAlgae;
        this.reefBackoff = false;
        this.shortJoinPath = false;
        addRequirements(driveSubsystem);
    }

    public PathfindingAutoAlignCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            Supplier<Double> heightSupplier,
            Supplier<Command> stageCommandSupplier,
            BooleanSupplier doneStagingSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            boolean reefBackoff,
            boolean shortJoinPath) {
        this.robotState = robotState;
        this.branchSupplier = branchSupplier;
        this.letterSupplier = letterSupplier;
        this.heightSupplier = heightSupplier;
        this.stageCommandSupplier = stageCommandSupplier;
        this.doneStagingSupplier = doneStagingSupplier;
        this.isRedAlliance = isRedAlliance;
        this.isAlgae = isAlgae;
        this.reefBackoff = reefBackoff;
        this.shortJoinPath = shortJoinPath;
        addRequirements(driveSubsystem);
    }

    private static PathPlannerPath buildJoinPath(
            Pose2d curPose, Pose2d joinPathStart, Pose2d joinPathEnd, boolean isTall) {
        final double kIntermediateFractionDefault = 0.6;
        final double kIntermediateFractionL4 = 0.2;
        final double kIntermediateFraction =
                isTall ? kIntermediateFractionL4 : kIntermediateFractionDefault;
        Pose2d intermediatePose = joinPathStart.interpolate(joinPathEnd, kIntermediateFraction);

        PathPlannerPath joinPath;
        if (MathHelpers.reverseInterpolate(
                        curPose.getTranslation(),
                        joinPathStart.getTranslation(),
                        joinPathEnd.getTranslation())
                <= 0.0) {
            // If we're far enough from the start point, add an extra point to come in from further
            // away.
            Pose2d secondPose = joinPathStart.interpolate(joinPathEnd, 0.1);
            joinPath =
                    new PathPlannerPath(
                            PathPlannerPath.waypointsFromPoses(
                                    joinPathStart, secondPose, intermediatePose, joinPathEnd),
                            pathfindConstraints,
                            new IdealStartingState(0.0, joinPathStart.getRotation()),
                            new GoalEndState(0.0, joinPathEnd.getRotation()));
            return joinPath;
        }
        joinPath =
                new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                joinPathStart, intermediatePose, joinPathEnd),
                        pathfindConstraints,
                        new IdealStartingState(0.0, joinPathStart.getRotation()),
                        new GoalEndState(0.0, joinPathEnd.getRotation()));

        return joinPath;
    }

    private PathPlannerPath buildJoinPath(Pose2d startPose, Pose2d endPose) {
        Pose2d curPose = robotState.getLatestFieldToRobot().getValue();
        joinPath = buildJoinPath(curPose, startPose, endPose, isTall());
        return joinPath;
    }

    private boolean isTall() {
        double height = heightSupplier == null ? 0.0 : heightSupplier.get();
        final double kSlowPathThreshold = 1.0;
        return height >= kSlowPathThreshold;
    }

    private PathConstraints getJoinPathConstraints() {
        PathConstraints constraints = isTall() ? slowJoinPathConstraints : joinPathConstraints;
        return constraints;
    }

    private IPathCallback buildPathCallback() {
        return new IPathCallback() {
            boolean updated = true;
            boolean stageCommandFinished = false;
            boolean readyToScore = false;
            Command stageCommand = buildStageCommand();
            boolean wasTall = isTall();

            private Command buildStageCommand() {
                Logger.recordOutput("PathfindingAutoAlignCommand/stageCommandFinished", false);
                return Commands.defer(stageCommandSupplier, Set.of())
                        .andThen(Commands.runOnce(this::setStageCommandFinished))
                        .until(() -> stageCommandFinished);
            }

            public void setStageCommandFinished() {
                Logger.recordOutput("PathfindingAutoAlignCommand/stageCommandFinished", true);
                stageCommandFinished = true;
                updated = true;
            }

            private boolean poseReadyToScore() {
                if (joinPath == null) return true;

                var pose = robotState.getLatestFieldToRobot().getValue();

                double angleDist =
                        Math.abs(
                                pose.getRotation()
                                        .minus(joinPath.getInitialHeading())
                                        .getRadians());
                boolean angleCondition = angleDist <= Units.degreesToRadians(6.0);
                Logger.recordOutput("PathfindingAutoAlignCommand/angleCondition", angleCondition);

                double cte =
                        MathHelpers.perpendicularDistanceToLine(
                                pose.getTranslation(),
                                joinPath.getWaypoints().get(1).anchor(),
                                joinPath.getWaypoints()
                                        .get(joinPath.getWaypoints().size() - 1)
                                        .anchor());
                boolean cteCondition = cte <= (isAlgae ? 0.15 : 0.1);
                Logger.recordOutput("PathfindingAutoAlignCommand/cteCondition", cteCondition);

                var velocity = robotState.getLatestFusedRobotRelativeChassisSpeed();
                boolean yVelocityCondition = Math.abs(velocity.vyMetersPerSecond) <= .3;
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/yVelocityCondition", yVelocityCondition);

                boolean thetaVelocityCondition = Math.abs(velocity.omegaRadiansPerSecond) <= .3;
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/thetaVelocityCondition",
                        thetaVelocityCondition);

                double t = RobotTime.getTimestampSeconds();
                boolean maxRollVelCondition =
                        robotState.getMaxAbsDriveRollAngularVelocityInRange(t - .3, t).orElse(0.0)
                                <= .3;
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/maxRollVelCondition", maxRollVelCondition);

                boolean maxPitchVelCondition =
                        robotState.getMaxAbsDrivePitchAngularVelocityInRange(t - .3, t).orElse(0.0)
                                <= .3;
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/maxPitchVelCondition", maxPitchVelCondition);

                double lastUsedMegatagTimestamp = robotState.lastUsedMegatagTimestamp();
                boolean visionAgeCondition = t - lastUsedMegatagTimestamp <= 0.5;
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/visionAgeCondition", visionAgeCondition);

                Pose2d megatagPose = robotState.lastUsedMegatagPose();
                boolean visionAccuracyCondition =
                        megatagPose.getTranslation().getDistance(pose.getTranslation()) <= 0.2;
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/visionAccuracyCondition",
                        visionAccuracyCondition);

                // Just send it in auto. We've curated the paths to maximize likelihood of scoring.
                if (DriverStation.isAutonomousEnabled()) return true;

                // Vision conditions
                if (!visionAgeCondition || !visionAccuracyCondition) {
                    return false;
                }

                // Conditions that apply to both coral and algae.
                if (!cteCondition
                        || (!angleCondition && branchSupplier.get() != ReefBranch.BARGE)
                        || !yVelocityCondition
                        || !thetaVelocityCondition) {
                    return false;
                }

                if (isAlgae || branchSupplier.get() == ReefBranch.BARGE || shortJoinPath) {
                    return true;
                }

                // Coral only conditions.
                if (!maxPitchVelCondition || !maxRollVelCondition) {
                    return false;
                }

                return true;
            }

            @Override
            public boolean markupUpdated() {
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/doneStagingSupplier",
                        doneStagingSupplier.getAsBoolean());
                if (!readyToScore) {
                    if (poseReadyToScore()
                            && stageCommandFinished
                            && doneStagingSupplier.getAsBoolean()) {
                        updated = true;
                        readyToScore = true;
                    }
                }
                if (wasTall != isTall()) {
                    wasTall = isTall();
                    updated = true;
                }
                return updated;
            }

            @Override
            public PathMarkup apply(PathPlannerPath path) {
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/doneStagingSupplier",
                        doneStagingSupplier.getAsBoolean());
                Logger.recordOutput("PathfindingAutoAlignCommand/readyToScore", readyToScore);
                Logger.recordOutput(
                        "PathfindingAutoAlignCommand/stageCommandFinishedInApply",
                        stageCommandFinished);

                updated = false;
                PathMarkup markup = new PathMarkup();
                markup.constraintsZones = new ArrayList<ConstraintsZone>();
                markup.eventMarkers = new ArrayList<EventMarker>();

                // Assume the 2nd to last point is the stitch point.
                int stitchIndex = Math.max(0, path.getWaypoints().size() - 2);
                PathConstraints joinPathConstraints = getJoinPathConstraints();
                if (isTall()) {
                    // Add a constraints zone with reduced accel to handle the top heavy robot case.
                    PathConstraints tallConstraints = slowPathfindConstraints;
                    markup.constraintsZones.add(
                            new ConstraintsZone(0, stitchIndex, tallConstraints));
                }
                if (readyToScore) {
                    // Make sure the stitch path obeys our limits.
                    markup.constraintsZones.add(
                            new ConstraintsZone(
                                    stitchIndex,
                                    path.getWaypoints().size() - 1,
                                    joinPathConstraints));

                    // Just drive, baby.
                    return markup;
                } else {
                    // If we aren't staged yet, instead plan on "stopping" at the stitchIndex. You
                    // can't actually stop in
                    // PathPlanner, but you can go really slowly...

                    // Reduce assumed speed and accel until we can stage.
                    PathConstraints adjusted =
                            new PathConstraints(
                                    0.001,
                                    joinPathConstraints.maxAccelerationMPSSq(),
                                    joinPathConstraints.maxAngularVelocityRadPerSec(),
                                    joinPathConstraints.maxAngularAccelerationRadPerSecSq(),
                                    joinPathConstraints.maxXAccelMPSSq(),
                                    joinPathConstraints.maxYAccelMPSSq());
                    markup.constraintsZones.add(
                            new ConstraintsZone(
                                    stitchIndex, path.getWaypoints().size() - 1, adjusted));
                }

                // Figure out when we should start staging.

                // Find the distance to the stage point.
                var allPoints = path.getAllPathPoints();
                int allPointsStageByIndex = 0;
                double allPointsStageByDistance = 0;
                for (int i = allPoints.size() - 1; i >= 0; --i) {
                    if (i > allPointsStageByIndex
                            && allPoints.get(i).waypointRelativePos <= stitchIndex + 1e-6) {
                        allPointsStageByIndex = i;
                        allPointsStageByDistance = allPoints.get(i).distanceAlongPath;
                        break;
                    }
                }

                final double kStageTime;
                if (branchSupplier.get() == ReefBranch.BARGE) {
                    kStageTime = 2.0;
                } else if (isAlgae && shortJoinPath) {
                    kStageTime = 0.1;
                } else {
                    kStageTime = 1.0;
                }
                final double kAssumedSpeed = pathfindConstraints.maxVelocityMPS() * .7;

                double neededDistance = kAssumedSpeed * kStageTime;

                if (allPointsStageByDistance > neededDistance) {
                    // We can wait to stage.
                    for (int i = allPointsStageByIndex; i >= 0; --i) {
                        var point = allPoints.get(i);
                        if (point.distanceAlongPath + neededDistance <= allPointsStageByDistance) {
                            markup.eventMarkers.add(
                                    new EventMarker(
                                            "Start Staging",
                                            point.waypointRelativePos,
                                            stageCommand));
                            break;
                        }
                    }
                } else {
                    markup.eventMarkers.add(new EventMarker("Start Staging", 0, stageCommand));
                }
                return markup;
            }
        };
    }

    private Pair<Pose2d, Pose2d> getJoinPathStartEnd(Pose2d curPose) {
        ReefBranch branch = branchSupplier.get();
        boolean letter = letterSupplier.get();
        boolean isRedAlliance = this.isRedAlliance.getAsBoolean();

        boolean auto = DriverStation.isAutonomous() || DriverStation.isDisabled();

        ScoringLocation targetLocation =
                isAlgae
                        ? ScoringLocation.getLocationAlgae(branch)
                        : ScoringLocation.getLocation(branch, letter);
        Pose2d targetPose =
                auto
                                && !(branch == ReefBranch.FEEDER
                                        || branch == ReefBranch.FEEDER_GROUND
                                        || branch == ReefBranch.FEEDER_GROUND_RETRY
                                        || branch == ReefBranch.BARGE)
                                && !isAlgae
                                && !reefBackoff
                        ? targetLocation.getPosePlusTransform(
                                Constants.kAutoAlignReefBuffer - Constants.kAutoAlignReefBufferAuto)
                        : targetLocation.getPose();
        Logger.recordOutput("PathfindingAutoAlignCommand/TargetPose", targetPose);
        Pose2d startPose = isRedAlliance ? FlippingUtil.flipFieldPose(targetPose) : targetPose;
        Pose2d endPose =
                isRedAlliance
                        ? FlippingUtil.flipFieldPose(targetLocation.getPoseWithoutReefBuffer())
                        : targetLocation.getPoseWithoutReefBuffer();
        if (shortJoinPath) {
            Transform2d shorterBuffer =
                    new Transform2d(
                            new Translation2d(
                                    isAlgae
                                            ? Constants.kAutoAlignReefBackoffDistance
                                            : Constants.kAutoAlignReefBackoffDistance,
                                    0.0),
                            Rotation2d.kZero);
            startPose = endPose.plus(shorterBuffer);
        }
        if (targetLocation == ScoringLocation.Barge) {
            endPose = startPose;
            startPose = endPose.plus(new Transform2d(new Translation2d(0.5, 0), Rotation2d.kZero));
        }

        if (DriverStation.isEnabled()
                && RobotState.onOpponentSide(isRedAlliance, curPose)
                && isAlgae) {
            startPose = FlippingUtil.flipFieldPose(startPose);
            endPose = FlippingUtil.flipFieldPose(endPose);
        }

        boolean isFeeder = branch == ReefBranch.FEEDER;
        if (isFeeder) {
            endPose =
                    startPose.plus(
                            new Transform2d(
                                    new Translation2d(
                                            (DriverStation.isAutonomous()
                                                            || DriverStation.isDisabled())
                                                    ? Constants.kAutoAlignFeederBufferAuto
                                                    : Constants.kAutoAlignFeederBufferTeleop,
                                            0.0),
                                    Rotation2d.kZero));
        } else if (reefBackoff) {
            endPose =
                    endPose.plus(
                            new Transform2d(
                                    new Translation2d(
                                            isAlgae
                                                    ? Constants.kAutoAlignAlgaeReefBackoffDistance
                                                    : Constants.kAutoAlignReefBackoffDistance,
                                            0.0),
                                    Rotation2d.kZero));
        } else if (branch == ReefBranch.FEEDER_GROUND || branch == ReefBranch.FEEDER_GROUND_RETRY) {
            curPose = isRedAlliance ? FlippingUtil.flipFieldPose(curPose) : curPose;
            if (curPose.getTranslation().getX() > 4.5) {
                endPose =
                        startPose.plus(
                                new Transform2d(
                                        new Translation2d(
                                                branch == ReefBranch.FEEDER_GROUND
                                                        ? Constants
                                                                .kAutoAlignFeederGroundBufferFirstPathAuto
                                                        : 0.2,
                                                0.0),
                                        Rotation2d.kZero));
                endPose =
                        new Pose2d(
                                endPose.getTranslation(),
                                isRedAlliance
                                        ? letter
                                                ? Rotation2d.fromDegrees(160)
                                                : Rotation2d.fromDegrees(200)
                                        : letter
                                                ? Rotation2d.fromDegrees(-20)
                                                : Rotation2d.fromDegrees(20));
            } else {
                endPose =
                        startPose.plus(
                                new Transform2d(
                                        new Translation2d(
                                                branch == ReefBranch.FEEDER_GROUND
                                                        ? Constants.kAutoAlignFeederGroundBufferAuto
                                                        : 0.15,
                                                0.0),
                                        Rotation2d.kZero));
            }
        }
        return new Pair<Pose2d, Pose2d>(startPose, endPose);
    }

    private boolean shouldUseJoinPath() {
        return branchSupplier.get() != ReefBranch.FEEDER
                && branchSupplier.get() != ReefBranch.FEEDER_GROUND
                && branchSupplier.get() != ReefBranch.FEEDER_GROUND_RETRY
                && !reefBackoff;
    }

    private PathConstraints getConstraints() {
        return shouldUseJoinPath() ? pathfindConstraints : pathfindToFeederConstraints;
    }

    @Override
    public void initialize() {
        Logger.recordOutput(
                "PathfindingAutoAlignCommand/Initialize", "Starting auto-align command");

        Logger.recordOutput("AutoScore/StartTime", Timer.getFPGATimestamp());

        Pose2d curPose = robotState.getLatestFieldToRobot().getValue();
        var startEnd = getJoinPathStartEnd(curPose);
        Pose2d startPose = startEnd.getFirst();
        Pose2d endPose = startEnd.getSecond();

        Logger.recordOutput("PathfindingAutoAlignCommand/EndPose", endPose);
        Logger.recordOutput("PathfindingAutoAlignCommand/StartPose", startPose);

        PathConstraints constraints = getConstraints();
        if (shouldUseJoinPath()) {
            int exclusiveTag = 0;
            if (isRedAlliance.getAsBoolean()) {
                if (robotState.onOpponentSide() && isAlgae) {
                    exclusiveTag = branchSupplier.get().getBlueTagId();
                } else {
                    exclusiveTag = branchSupplier.get().getRedTagId();
                }
            } else {
                if (robotState.onOpponentSide() && isAlgae) {
                    exclusiveTag = branchSupplier.get().getRedTagId();
                } else {
                    exclusiveTag = branchSupplier.get().getBlueTagId();
                }
            }
            joinPath = buildJoinPath(startPose, endPose);
            robotState.setExclusiveTag(exclusiveTag);
            innerCommand =
                    AutoBuilder.pathfindThenFollowPath(joinPath, constraints, buildPathCallback());
        } else {
            innerCommand = AutoBuilder.pathfindToPose(endPose, constraints);
        }

        innerCommand.initialize();
    }

    @Override
    public void execute() {
        if (innerCommand != null && !innerCommand.isFinished()) {
            innerCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return innerCommand == null || innerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        robotState.clearExclusiveTag();
        if (interrupted && innerCommand != null) {
            Logger.recordOutput(
                    "PathfindingAutoAlignCommand/End", "Interrupted! Cancelling inner command.");
            innerCommand.end(true);
        } else if (innerCommand != null) {
            innerCommand.end(false);
            Logger.recordOutput("PathfindingAutoAlignCommand/End", "Finished successfully.");
        }
    }

    public static class WarmupCommand extends Command {
        Pose2d start;
        PathPlannerPath joinPath;
        Pose2d endPose;
        PathConstraints constraints;

        public WarmupCommand(Pose2d start, PathPlannerPath joinPath, PathConstraints constraints) {
            this.start = start;
            this.joinPath = joinPath;
            endPose = joinPath.getPathPoses().get(joinPath.getPathPoses().size() - 1);
            this.constraints = constraints;
            Pathfinding.ensureInitialized();
        }

        public WarmupCommand(Pose2d start, Pose2d endPose, PathConstraints constraints) {
            this.start = start;
            this.joinPath = null;
            this.endPose = endPose;
            this.constraints = constraints;
            Pathfinding.ensureInitialized();
        }

        public Pose2d getFinalPose() {
            return endPose;
        }

        @Override
        public void initialize() {
            if (joinPath != null) {
                Pathfinding.setProblem(start, joinPath, Translation2d.kZero, constraints);
            } else {
                Pathfinding.setProblem(start, endPose, Translation2d.kZero, constraints);
            }
        }

        @Override
        public boolean isFinished() {
            return Pathfinding.isNewPathAvailable();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                System.out.println("[PathfindingAutoAlignCommand.WarmupCommand] INTERRUPTED");
            } else {
                System.out.println(
                        "[PathfindingAutoAlignCommand.WarmupCommand] Warmup from pose "
                                + start
                                + " to "
                                + endPose
                                + " finished");
            }
        }
    }

    public WarmupCommand getWarmupCommand(Pose2d start) {
        var startEnd = getJoinPathStartEnd(start);
        if (shouldUseJoinPath()) {
            var joinPath =
                    buildJoinPath(start, startEnd.getFirst(), startEnd.getSecond(), isTall());
            return new WarmupCommand(start, joinPath, getConstraints());
        }
        return new WarmupCommand(start, startEnd.getSecond(), getConstraints());
    }
}
