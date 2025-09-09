package com.team254.frc2025.auto;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.auto.AutoModeSelector.FeederStrategy;
import com.team254.frc2025.auto.AutoModeSelector.ScoringPosition;
import com.team254.frc2025.auto.AutoModeSelector.StartingPosition;
import com.team254.frc2025.commands.PathfindingAutoAlignCommand;
import com.team254.frc2025.controlboard.ModalControls.Mode;
import com.team254.frc2025.factories.AutoFactory;
import com.team254.frc2025.factories.SuperstructureFactory;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.commands.ChezySequenceCommandGroup;
import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.pathplanner.util.FlippingUtil;
import com.team254.lib.reefscape.ReefBranch;
import com.team254.lib.reefscape.ScoringLocation;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class PathfindingWarmupCommand extends ChezySequenceCommandGroup {

    protected final RobotContainer container;
    protected final String scoreSequence;
    protected final String levelSequence;
    protected Pose2d startingPose = null;
    protected BooleanSupplier isRedAlliance;
    static final double kPathfindToIceCreamSpeedScalingFactor = 0.7;
    static final double kPathfindToIceCreamAccelScalingFactor = 0.7;
    static final double kDistanceAdvantageTolerance = 0.0;
    private static final PathConstraints pathfindToIceCreamConstraints =
            new PathConstraints(
                    Constants.DriveConstants.kDriveMaxSpeed * kPathfindToIceCreamSpeedScalingFactor,
                    Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                            * kPathfindToIceCreamAccelScalingFactor,
                    Constants.DriveConstants.kDriveMaxAngularRate
                            * kPathfindToIceCreamSpeedScalingFactor,
                    Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            * kPathfindToIceCreamAccelScalingFactor,
                    Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                            * kPathfindToIceCreamAccelScalingFactor,
                    Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                            * kPathfindToIceCreamAccelScalingFactor);
    protected List<PathfindingAutoAlignCommand> warmupCommands =
            new ArrayList<PathfindingAutoAlignCommand>();

    public PathfindingWarmupCommand(
            RobotContainer container,
            String scoreSequence,
            String levelSequence,
            StartingPosition startPosition,
            int iceCreamCnt,
            FeederStrategy feederStrategy,
            BooleanSupplier isRedAlliance) {
        this.container = container;
        this.scoreSequence = scoreSequence.toLowerCase();
        this.levelSequence = levelSequence;
        this.isRedAlliance = isRedAlliance;
        this.startingPose = startPosition.getStartingPose();

        ArrayList<Pose2d> remainingIceCream =
                (startPosition == StartingPosition.LEFT_BARGE)
                        ? new ArrayList<>(
                                Arrays.asList(
                                        ScoringLocation.IceCreamLeft.getPose(),
                                        ScoringLocation.IceCreamMiddle.getPose(),
                                        ScoringLocation.IceCreamRight.getPose()))
                        : new ArrayList<>(
                                Arrays.asList(
                                        ScoringLocation.IceCreamRight.getPose(),
                                        ScoringLocation.IceCreamMiddle.getPose(),
                                        ScoringLocation.IceCreamLeft.getPose()));
        if (isRedAlliance.getAsBoolean()) {
            for (int i = 0; i < remainingIceCream.size(); i++) {
                remainingIceCream.set(i, FlippingUtil.flipFieldPose(remainingIceCream.get(i)));
            }
        }
        addCommands(
                new InstantCommand(() -> container.getModalControls().forceSetMode(Mode.CORAL)));
        for (int i = 0; i < this.scoreSequence.length(); i++) {
            ScoringPosition scorePosition =
                    AutoModeSelector.getScoringPosition(
                            this.scoreSequence.charAt(i), this.levelSequence.charAt(i));
            Pair<ReefBranch, Boolean> position =
                    ScoringLocation.getSelectors(scorePosition.getScoringLocation());
            addCommands(
                    new InstantCommand(
                            () ->
                                    Logger.recordOutput(
                                            "Current Auto Position Reef Branch",
                                            scorePosition.getDisplayName())));
            if (scoreSequence.charAt(i) == 'A' || scoreSequence.charAt(i) == 'B') {
                addCommands(
                        new InstantCommand(
                                        () ->
                                                container
                                                        .getModalSuperstructureTriggers()
                                                        .getDefaultRobotWide()
                                                        .set(false))
                                .withName("Set tight auto"));
            }
            if (ScoringLocation.isAlgaeReefIntakeLocation(scorePosition.getScoringLocation())) {
                addCommands(
                        new InstantCommand(
                                () -> container.getModalControls().forceSetMode(Mode.ALGAECLIMB)));
                boolean algaeIntakeAfterBackoff =
                        (i > 0 && this.levelSequence.charAt(i - 1) != '*');
                if (algaeIntakeAfterBackoff) {
                    addCommands(new InstantCommand(() -> Pathfinding.setBackoffObstacles()));
                }
                addCommands(
                        Commands.waitUntil(() -> !container.getClaw().hasAlgae())
                                .andThen(Commands.waitUntil(() -> container.getClaw().hasAlgae()))
                                .deadlineFor(
                                        AutoFactory.getPathfindToAlgaeIntake(
                                                container,
                                                () -> position.getFirst(),
                                                () -> position.getSecond(),
                                                algaeIntakeAfterBackoff,
                                                warmupCommands))
                                .andThen(
                                        // sets the teleop obstacles after it finishes
                                        AutoFactory.getPathfindBackoffFromReefCommand(
                                                container.getDriveSubsystem(),
                                                container.getRobotState(),
                                                () -> position.getFirst(),
                                                () -> position.getSecond(),
                                                isRedAlliance,
                                                true,
                                                warmupCommands)));
            } else {
                addCommands(
                        AutoFactory.getPathfindToReefCommand(
                                        container.getDriveSubsystem(),
                                        container.getRobotState(),
                                        () -> position.getFirst(),
                                        () -> position.getSecond(),
                                        () -> scorePosition.getState().getElevatorHeightMeters(),
                                        () ->
                                                container
                                                        .getModalSuperstructureTriggers()
                                                        .setFutureStateAndWaitUntilDesiredCommand(
                                                                scorePosition.getState(),
                                                                scoreSequence)
                                                        .withName("Stagingcommand"),
                                        () ->
                                                isCoralStageState(
                                                                container
                                                                        .getStateMachine()
                                                                        .getCurrentState())
                                                        || (isCoralStageState(
                                                                        container
                                                                                .getStateMachine()
                                                                                .getDesiredState())
                                                                && container
                                                                                .getRobotState()
                                                                                .getElevatorHeightMeters()
                                                                        >= scorePosition
                                                                                .getState()
                                                                                .getContinueAutoAlignElevatorPosition()),
                                        isRedAlliance,
                                        false,
                                        false,
                                        warmupCommands)
                                .alongWith(
                                        new ConditionalCommand(
                                                Commands.none(),
                                                SuperstructureFactory.groundIntakeNoIndexerNoEnd(
                                                                container)
                                                        .withName("run intake during path to reef")
                                                        .asProxy()
                                                        .until(
                                                                () ->
                                                                        container
                                                                                        .getCoralStateTracker()
                                                                                        .getCurrentPosition()
                                                                                != CoralPosition
                                                                                        .NONE),
                                                () ->
                                                        container
                                                                        .getCoralStateTracker()
                                                                        .getCurrentPosition()
                                                                != CoralPosition.NONE))
                                .until(
                                        () ->
                                                container.getStateMachine().getDesiredState()
                                                                != null
                                                        && container
                                                                .getStateMachine()
                                                                .getDesiredState()
                                                                .equals(
                                                                        scorePosition
                                                                                .getState()
                                                                                .getAvoidDescoreState())
                                                        && ((scorePosition.getState()
                                                                                == SuperstructureState
                                                                                        .STAGE_CORAL_L4
                                                                        && container
                                                                                        .getRobotState()
                                                                                        .getElevatorHeightMeters()
                                                                                >= Constants
                                                                                        .ElevatorConstants
                                                                                        .kElevatorL4ClearanceHeightMeters)
                                                                || (scorePosition.getState()
                                                                                != SuperstructureState
                                                                                        .STAGE_CORAL_L4
                                                                        && container
                                                                                        .getRobotState()
                                                                                        .getWristRadians()
                                                                                >= Constants
                                                                                        .WristConstants
                                                                                        .kWristDescoreClearancePositionRadians))));
            }
            if (scoreSequence.charAt(i) == 'A' || scoreSequence.charAt(i) == 'B') {
                addCommands(
                        new InstantCommand(
                                        () ->
                                                container
                                                        .getModalSuperstructureTriggers()
                                                        .getDefaultRobotWide()
                                                        .set(true))
                                .withName("Set wide auto"));
            }
            addCommands(
                    new InstantCommand(
                            () ->
                                    Logger.recordOutput(
                                            "Current Auto Position Reef Branch", "null")));

            if (i != this.scoreSequence.length() - 1
                    && (i + iceCreamCnt + 1 >= this.scoreSequence.length())) {
                addCommands(
                        new InstantCommand(() -> Pathfinding.setTeleopObstacles())
                                .andThen(
                                        Commands.defer(
                                                () -> {
                                                    int closestIceCreamIdx = 0;
                                                    // defer to use actual robot pose
                                                    Translation2d scorePosTranslation =
                                                            container
                                                                    .getRobotState()
                                                                    .getLatestFieldToRobot()
                                                                    .getValue()
                                                                    .getTranslation();
                                                    for (int j = 1;
                                                            j < remainingIceCream.size();
                                                            j++) {
                                                        if (scorePosTranslation.getDistance(
                                                                        remainingIceCream
                                                                                .get(j)
                                                                                .getTranslation())
                                                                < scorePosTranslation.getDistance(
                                                                                remainingIceCream
                                                                                        .get(
                                                                                                closestIceCreamIdx)
                                                                                        .getTranslation())
                                                                        - kDistanceAdvantageTolerance) {
                                                            closestIceCreamIdx = j;
                                                        }
                                                    }
                                                    Pose2d closestIceCreamPose =
                                                            new Pose2d(
                                                                    remainingIceCream
                                                                            .get(closestIceCreamIdx)
                                                                            .getTranslation(),
                                                                    scorePosTranslation
                                                                            .minus(
                                                                                    remainingIceCream
                                                                                            .get(
                                                                                                    closestIceCreamIdx)
                                                                                            .getTranslation())
                                                                            .getAngle());
                                                    remainingIceCream.remove(closestIceCreamIdx);

                                                    Command iceCreamIntakeCmd =
                                                            AutoBuilder.pathfindToPose(
                                                                            closestIceCreamPose,
                                                                            pathfindToIceCreamConstraints)
                                                                    .alongWith(
                                                                            new WaitUntilCommand(
                                                                                            () ->
                                                                                                    container
                                                                                                                    .getCoralStateTracker()
                                                                                                                    .getCurrentPosition()
                                                                                                            == CoralPosition
                                                                                                                    .NONE)
                                                                                    .andThen(
                                                                                            SuperstructureFactory
                                                                                                    .groundIntake(
                                                                                                            container)
                                                                                                    .asProxy()
                                                                                                    .onlyWhile(
                                                                                                            () ->
                                                                                                                    !((container
                                                                                                                                            .getCoralStateTracker()
                                                                                                                                            .getCurrentPosition()
                                                                                                                                    == CoralPosition
                                                                                                                                            .AT_FIRST_INDEXER)
                                                                                                                            && (container
                                                                                                                                            .getStateMachine()
                                                                                                                                            .getCurrentState()
                                                                                                                                    != SuperstructureState
                                                                                                                                            .STOW_CORAL)))));
                                                    return iceCreamIntakeCmd;
                                                },
                                                Set.of()))
                                .andThen(new InstantCommand(() -> Pathfinding.setAutoObstacles())));
            } else {
                Command pathfindingCmd = Commands.none();
                if (i < this.levelSequence.length() - 1 && levelSequence.charAt(i + 1) == '*') {
                    pathfindingCmd =
                            new InstantCommand(
                                            () ->
                                                    container
                                                            .getModalControls()
                                                            .forceSetMode(Mode.ALGAECLIMB))
                                    .andThen(
                                            AutoFactory.getPathfindBackoffFromReefCommand(
                                                    container.getDriveSubsystem(),
                                                    container.getRobotState(),
                                                    () -> position.getFirst(),
                                                    () -> position.getSecond(),
                                                    isRedAlliance,
                                                    false,
                                                    warmupCommands));
                } else if (feederStrategy == FeederStrategy.FUNNEL) {
                    pathfindingCmd =
                            AutoFactory.getPathfindToFeederCommand(
                                    container.getDriveSubsystem(),
                                    container.getRobotState(),
                                    () -> startPosition == StartingPosition.LEFT_BARGE,
                                    isRedAlliance,
                                    warmupCommands);
                } else if (feederStrategy == FeederStrategy.GROUND) {
                    pathfindingCmd =
                            AutoFactory.getPathfindToFeederGroundCommand(
                                            container.getDriveSubsystem(),
                                            container.getRobotState(),
                                            () -> startPosition == StartingPosition.LEFT_BARGE,
                                            isRedAlliance,
                                            warmupCommands)
                                    .raceWith(
                                            new WaitUntilCommand(
                                                            () ->
                                                                    container
                                                                                    .getCoralStateTracker()
                                                                                    .getCurrentPosition()
                                                                            == CoralPosition.NONE)
                                                    .andThen(
                                                            Commands.waitSeconds(0.5)
                                                                    .andThen(
                                                                            Commands.waitUntil(
                                                                                    () ->
                                                                                            container
                                                                                                                    .getCoralStateTracker()
                                                                                                                    .getCurrentPosition()
                                                                                                            != CoralPosition
                                                                                                                    .NONE
                                                                                                    || container
                                                                                                            .getIntakePivot()
                                                                                                            .hasCoralAtIntake()))
                                                                    .deadlineFor(
                                                                            SuperstructureFactory
                                                                                    .groundIntakeNoIndexerNoEnd(
                                                                                            container)
                                                                                    .asProxy())));
                }
                addCommands(pathfindingCmd);
            }
        }
    }

    public ScoringLocation getFirstScoringLocation() {
        if (scoreSequence.length() == 0 || levelSequence.length() == 0) {
            return ScoringLocation.A;
        }
        return AutoModeSelector.getScoringPosition(scoreSequence.charAt(0), levelSequence.charAt(0))
                .getScoringLocation();
    }

    public Optional<Pose2d> getStartingPose() {
        return Optional.ofNullable(startingPose);
    }

    private boolean isCoralStageState(SuperstructureState state) {
        return state == SuperstructureState.STAGE_CORAL_L2
                || state == SuperstructureState.STAGE_CORAL_L3
                || state == SuperstructureState.STAGE_CORAL_L4;
    }

    public Command getWarmupCommand() {
        ChezySequenceCommandGroup warmupCommand = new ChezySequenceCommandGroup();
        Pose2d startPose = startingPose;
        warmupCommand.addCommands(
                new InstantCommand(
                        () -> Logger.recordOutput("warmup commands size", warmupCommands.size())));
        for (var cmd : warmupCommands) {
            PathfindingAutoAlignCommand.WarmupCommand warmup = cmd.getWarmupCommand(startPose);
            warmupCommand.addCommands(warmup);
            startPose = warmup.getFinalPose();
        }
        return warmupCommand.ignoringDisable(true);
    }

    public Command getFirstWarmupCommand(Pose2d startPose) {
        if (warmupCommands.isEmpty()) return Commands.none();
        return warmupCommands.get(0).getWarmupCommand(startPose).ignoringDisable(true);
    }
}
