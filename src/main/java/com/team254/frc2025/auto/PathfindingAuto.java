package com.team254.frc2025.auto;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.auto.AutoModeSelector.FeederStrategy;
import com.team254.frc2025.auto.AutoModeSelector.ScoringPosition;
import com.team254.frc2025.auto.AutoModeSelector.StartingPosition;
import com.team254.frc2025.commands.AutoAlignToPoseCommand;
import com.team254.frc2025.commands.PathfindingAutoAlignCommand;
import com.team254.frc2025.controlboard.ModalControls.Mode;
import com.team254.frc2025.factories.AutoFactory;
import com.team254.frc2025.factories.SuperstructureFactory;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.commands.ChezyRepeatCommand;
import com.team254.lib.commands.ChezySequenceCommandGroup;
import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.pathplanner.util.FlippingUtil;
import com.team254.lib.reefscape.ReefBranch;
import com.team254.lib.reefscape.ScoringLocation;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class PathfindingAuto extends ChezySequenceCommandGroup {

    protected final RobotContainer container;
    protected final String scoreSequence;
    protected final String levelSequence;
    protected final StartingPosition startingPosition;
    protected Pose2d startingPose = null;
    protected final int iceCreamCnt;
    protected final FeederStrategy feederStrategy;
    protected final BooleanSupplier isRedAlliance;

    ScoringPosition lastScoredPosition = null;

    protected boolean scoredL3orL2 = false;

    private int currentStep = 0;
    private boolean retrying = false;
    private final List<Pose2d> remainingIceCream;

    protected final List<PathfindingAutoAlignCommand> warmupCommands = new ArrayList<>();

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

    public PathfindingAuto(
            RobotContainer container,
            String scoreSequence,
            String levelSequence,
            StartingPosition startingPosition,
            int iceCreamCnt,
            FeederStrategy feederStrategy,
            BooleanSupplier isRedAlliance) {
        this.container = container;
        this.scoreSequence = scoreSequence.toLowerCase();
        this.levelSequence = levelSequence;
        this.startingPosition = startingPosition;
        this.startingPose = startingPosition.getStartingPose();
        this.iceCreamCnt = iceCreamCnt;
        this.feederStrategy = feederStrategy;
        this.isRedAlliance = isRedAlliance;

        if (startingPosition == StartingPosition.LEFT_BARGE) {
            this.remainingIceCream =
                    new ArrayList<>(
                            Arrays.asList(
                                    ScoringLocation.IceCreamLeft.getPose(),
                                    ScoringLocation.IceCreamMiddle.getPose(),
                                    ScoringLocation.IceCreamRight.getPose()));
        } else {
            this.remainingIceCream =
                    new ArrayList<>(
                            Arrays.asList(
                                    ScoringLocation.IceCreamRight.getPose(),
                                    ScoringLocation.IceCreamMiddle.getPose(),
                                    ScoringLocation.IceCreamLeft.getPose()));
        }
        if (isRedAlliance.getAsBoolean()) {
            for (int i = 0; i < remainingIceCream.size(); i++) {
                remainingIceCream.set(i, FlippingUtil.flipFieldPose(remainingIceCream.get(i)));
            }
        }

        addCommands(
                new InstantCommand(() -> container.getModalControls().forceSetMode(Mode.CORAL)));
        addCommands(new ChezyRepeatCommand(getNextAction()));
    }

    public Command getNextAction() {
        return new DeferredCommand(
                () -> {
                    Logger.recordOutput("Current Step", currentStep);
                    if (currentStep >= scoreSequence.length()) {
                        return Commands.none();
                    }

                    char currentScore = scoreSequence.charAt(currentStep);
                    char currentLevel = levelSequence.charAt(currentStep);
                    ScoringPosition scoringPosition =
                            AutoModeSelector.getScoringPosition(currentScore, currentLevel);
                    Pair<ReefBranch, Boolean> position =
                            ScoringLocation.getSelectors(scoringPosition.getScoringLocation());

                    Logger.recordOutput(
                            "Current Auto Position Reef Branch", scoringPosition.getDisplayName());

                    ChezySequenceCommandGroup stepChain = new ChezySequenceCommandGroup();

                    if (currentScore == 'A' || currentScore == 'B') {
                        stepChain.addCommands(
                                new InstantCommand(
                                                () ->
                                                        container
                                                                .getModalSuperstructureTriggers()
                                                                .getDefaultRobotWide()
                                                                .set(false))
                                        .withName("Set tight auto"));
                    }

                    if (ScoringLocation.isAlgaeReefIntakeLocation(
                            scoringPosition.getScoringLocation())) {
                        stepChain.addCommands(
                                new InstantCommand(
                                        () ->
                                                container
                                                        .getModalControls()
                                                        .forceSetMode(Mode.ALGAECLIMB)));
                        boolean algaeIntakeAfterBackoff =
                                (currentStep > 0
                                        && levelSequence.charAt(currentStep - 1) != '*'
                                        && lastScoredPosition != null);
                        if (algaeIntakeAfterBackoff) {
                            stepChain.addCommands(
                                    new InstantCommand(
                                            () ->
                                                    container
                                                            .getModalControls()
                                                            .forceSetMode(Mode.ALGAECLIMB)));
                        }
                        stepChain.addCommands(
                                new InstantCommand(
                                        () ->
                                                Logger.recordOutput(
                                                        "Current Segment", "Algae Intake")));
                        stepChain.addCommands(
                                getAlgaeIntakeCommand(
                                        ScoringLocation.getSelectors(
                                                lastScoredPosition.getScoringLocation()),
                                        algaeIntakeAfterBackoff));
                        stepChain.addCommands(new InstantCommand(() -> currentStep++));
                        return stepChain;
                    } else {
                        stepChain.addCommands(
                                new InstantCommand(
                                        () ->
                                                Logger.recordOutput(
                                                        "Current Segment", "Reef Score")));
                        stepChain.addCommands(getReefScoringCommand(scoringPosition, position));
                    }

                    if (currentScore == 'A' || currentScore == 'B') {
                        stepChain.addCommands(
                                new InstantCommand(
                                                () ->
                                                        container
                                                                .getModalSuperstructureTriggers()
                                                                .getDefaultRobotWide()
                                                                .set(true))
                                        .withName("Set wide auto"));
                    }

                    stepChain.addCommands(
                            new InstantCommand(
                                    () ->
                                            Logger.recordOutput(
                                                    "Current Auto Position Reef Branch", "null")));

                    if (currentStep != scoreSequence.length() - 1
                            && (currentStep + iceCreamCnt + 1 >= scoreSequence.length())) {
                        stepChain.addCommands(
                                new InstantCommand(
                                        () ->
                                                Logger.recordOutput(
                                                        "Current Segment", "Ice Cream Intake")));
                        stepChain.addCommands(
                                new InstantCommand(() -> Pathfinding.setTeleopObstacles())
                                        .andThen(getIceCreamIntakeCommand())
                                        .andThen(
                                                new InstantCommand(
                                                        () -> Pathfinding.setAutoObstacles())));
                    } else {
                        stepChain.addCommands(
                                new InstantCommand(
                                        () ->
                                                Logger.recordOutput(
                                                        "Current Segment",
                                                        "Step Pathfinding Command")));
                        stepChain.addCommands(
                                Commands.defer(
                                        () -> getPathfindingCommandForStep(currentStep), Set.of()));
                    }

                    stepChain.addCommands(new InstantCommand(() -> currentStep++));

                    return stepChain;
                },
                Set.of(container.getDriveSubsystem()));
    }

    private Command getAlgaeIntakeCommand(
            Pair<ReefBranch, Boolean> position, boolean algaeIntakeAfterBackoff) {
        Command autoAlignToAlgaeSlap =
                Commands.defer(
                        () -> {
                            Pose2d currentPose =
                                    container.getRobotState().getLatestFieldToRobot().getValue();
                            Pose2d desiredPose =
                                    currentPose.plus(
                                            ScoringLocation.getLocationAlgae(position.getFirst())
                                                    .getPoseWithoutReefBufferPlusTransform(0.0)
                                                    .minus(
                                                            ScoringLocation.getLocation(
                                                                            position.getFirst(),
                                                                            position.getSecond())
                                                                    .getPoseWithoutReefBufferPlusTransform(
                                                                            0.0)));
                            return new AutoAlignToPoseCommand(
                                    container.getDriveSubsystem(),
                                    container.getRobotState(),
                                    desiredPose,
                                    1.0);
                        },
                        Set.of(container.getDriveSubsystem()));

        Command autoAlignBackoff =
                Commands.defer(
                        () -> {
                            Pose2d currentPose =
                                    container.getRobotState().getLatestFieldToRobot().getValue();
                            Pose2d desiredPose =
                                    currentPose.plus(new Transform2d(-0.3, 0.0, Rotation2d.kZero));
                            return new AutoAlignToPoseCommand(
                                    container.getDriveSubsystem(),
                                    container.getRobotState(),
                                    desiredPose,
                                    0.4);
                        },
                        Set.of(container.getDriveSubsystem()));

        Command autoAlignBackin =
                Commands.defer(
                        () -> {
                            Pose2d currentPose =
                                    container.getRobotState().getLatestFieldToRobot().getValue();
                            Pose2d desiredPose =
                                    currentPose.plus(new Transform2d(0.2, 0.0, Rotation2d.kZero));
                            return new AutoAlignToPoseCommand(
                                    container.getDriveSubsystem(),
                                    container.getRobotState(),
                                    desiredPose,
                                    0.5);
                        },
                        Set.of(container.getDriveSubsystem()));
        Command disableVision =
                new InstantCommand(() -> container.getVisionSubsystem().setUseVision(false));
        return new ChezySequenceCommandGroup(
                        Commands.waitUntil(
                                () ->
                                        container.getRobotState().getWristRadians()
                                                > Constants.WristConstants
                                                        .kClearL3DescoreForAlgaePositionRadians),
                        disableVision,
                        autoAlignToAlgaeSlap,
                        container
                                .getModalSuperstructureTriggers()
                                .slapAlgaeCommand()
                                .alongWith(
                                        new ChezySequenceCommandGroup(
                                                Commands.waitUntil(
                                                        () ->
                                                                container
                                                                                .getRobotState()
                                                                                .getWristRadians()
                                                                        < Constants.WristConstants
                                                                                .kWristL2SlapRadians),
                                                autoAlignBackoff)),
                        container
                                .getModalSuperstructureTriggers()
                                .setStateAndWaitCommand(
                                        SuperstructureState.GROUND_ALGAE_INTAKE,
                                        "move down to intake pose"),
                        autoAlignBackin)
                .finallyDo(() -> container.getVisionSubsystem().setUseVision(true));
    }

    private Command getReefScoringCommand(
            ScoringPosition scoringPosition, Pair<ReefBranch, Boolean> position) {
        Command pathfindToReef =
                (Commands.waitSeconds(1.5)
                                .andThen(
                                        Commands.waitUntil(
                                                () ->
                                                        container
                                                                                .getCoralStateTracker()
                                                                                .getCurrentPosition()
                                                                        == CoralPosition.NONE
                                                                && container
                                                                                .getStateMachine()
                                                                                .getCurrentState()
                                                                        == SuperstructureState
                                                                                .STOW_CORAL))
                                .andThen(
                                        new InstantCommand(
                                                () -> {
                                                    currentStep--;
                                                    retrying = true;
                                                })))
                        .raceWith(
                                new ChezySequenceCommandGroup(
                                        new InstantCommand(() -> retrying = false),
                                        (AutoFactory.getPathfindToReefCommand(
                                                                container.getDriveSubsystem(),
                                                                container.getRobotState(),
                                                                () -> position.getFirst(),
                                                                () -> position.getSecond(),
                                                                () ->
                                                                        container
                                                                                .getRobotState()
                                                                                .getElevatorHeightMeters(),
                                                                () ->
                                                                        container
                                                                                .getModalSuperstructureTriggers()
                                                                                .setFutureStateAndWaitUntilDesiredCommand(
                                                                                        scoringPosition
                                                                                                .getState(),
                                                                                        scoreSequence)
                                                                                .withName(
                                                                                        "Stagingcommand"),
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
                                                                                                >= scoringPosition
                                                                                                        .getState()
                                                                                                        .getContinueAutoAlignElevatorPosition()),
                                                                isRedAlliance,
                                                                false,
                                                                false,
                                                                warmupCommands)
                                                        .deadlineFor(
                                                                new ConditionalCommand(
                                                                        Commands.none(),
                                                                        SuperstructureFactory
                                                                                .groundIntakeNoIndexerNoEnd(
                                                                                        container)
                                                                                .withName(
                                                                                        "run intake during path to reef")
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
                                                                                        != CoralPosition
                                                                                                .NONE)))
                                                .repeatedly()));
        return new ChezySequenceCommandGroup(
                pathfindToReef.raceWith(
                        new ChezySequenceCommandGroup(
                                Commands.waitUntil(
                                        () ->
                                                container.getStateMachine().getDesiredState()
                                                                != null
                                                        && (container
                                                                .getStateMachine()
                                                                .getDesiredState()
                                                                .equals(
                                                                        scoringPosition
                                                                                .getState()
                                                                                .getAvoidDescoreState()))),
                                new InstantCommand(() -> scoredL3orL2 = true)
                                        .onlyIf(
                                                () ->
                                                        scoringPosition.getState()
                                                                        == SuperstructureState
                                                                                .STAGE_CORAL_L2
                                                                || scoringPosition.getState()
                                                                        == SuperstructureState
                                                                                .STAGE_CORAL_L3))),
                new InstantCommand(() -> lastScoredPosition = scoringPosition));
    }

    private boolean isCoralStageState(SuperstructureState state) {
        return state == SuperstructureState.STAGE_CORAL_L2
                || state == SuperstructureState.STAGE_CORAL_L3
                || state == SuperstructureState.STAGE_CORAL_L4;
    }

    private Command getIceCreamIntakeCommand() {
        return Commands.defer(
                () -> {
                    Translation2d scorePosTranslation =
                            container
                                    .getRobotState()
                                    .getLatestFieldToRobot()
                                    .getValue()
                                    .getTranslation();
                    int closestIceCreamIdx = 0;
                    for (int j = 1; j < remainingIceCream.size(); j++) {
                        if (scorePosTranslation.getDistance(
                                        remainingIceCream.get(j).getTranslation())
                                < scorePosTranslation.getDistance(
                                                remainingIceCream
                                                        .get(closestIceCreamIdx)
                                                        .getTranslation())
                                        - kDistanceAdvantageTolerance) {
                            closestIceCreamIdx = j;
                        }
                    }
                    Pose2d closestIceCreamPose =
                            new Pose2d(
                                    remainingIceCream.get(closestIceCreamIdx).getTranslation(),
                                    scorePosTranslation
                                            .minus(
                                                    remainingIceCream
                                                            .get(closestIceCreamIdx)
                                                            .getTranslation())
                                            .getAngle());
                    remainingIceCream.remove(closestIceCreamIdx);

                    Command pathfindToIceCream =
                            AutoBuilder.pathfindToPose(
                                    closestIceCreamPose, pathfindToIceCreamConstraints);
                    Command intakeCommand =
                            new WaitUntilCommand(
                                            () ->
                                                    container
                                                                    .getCoralStateTracker()
                                                                    .getCurrentPosition()
                                                            == CoralPosition.NONE)
                                    .andThen(
                                            SuperstructureFactory.groundIntakeNoEnd(container)
                                                    .asProxy()
                                                    .onlyWhile(
                                                            () ->
                                                                    !(container
                                                                                            .getCoralStateTracker()
                                                                                            .getCurrentPosition()
                                                                                    == CoralPosition
                                                                                            .AT_FIRST_INDEXER
                                                                            && container
                                                                                            .getStateMachine()
                                                                                            .getCurrentState()
                                                                                    != SuperstructureState
                                                                                            .STOW_CORAL)));
                    return pathfindToIceCream.alongWith(intakeCommand);
                },
                Set.of());
    }

    private Command getPathfindingCommandForStep(int stepIndex) {
        Command pathfindingCmd = Commands.none();
        var remainingLevelSequence = levelSequence.substring(currentStep + 1);
        Logger.recordOutput("Remaining Level Sequence", remainingLevelSequence);
        if (currentStep < levelSequence.length() - 1
                && levelSequence.charAt(currentStep + 1) == '*') {
            return Commands.none();
        } else if ((remainingLevelSequence.contains("*") && (scoredL3orL2))
                && Timer.getFPGATimestamp() - container.getRobotState().getAutoStartTime() > 12.0) {
            int nextStar = remainingLevelSequence.indexOf("*");
            currentStep += nextStar;
            return Commands.none();
        } else if (feederStrategy == FeederStrategy.FUNNEL) {
            pathfindingCmd =
                    AutoFactory.getPathfindToFeederCommand(
                            container.getDriveSubsystem(),
                            container.getRobotState(),
                            () -> startingPosition == StartingPosition.LEFT_BARGE,
                            isRedAlliance,
                            warmupCommands);
        } else if (feederStrategy == FeederStrategy.GROUND) {
            pathfindingCmd =
                    Commands.either(
                                    AutoFactory.getPathfindToFeederGroundRetryCommand(
                                            container.getDriveSubsystem(),
                                            container.getRobotState(),
                                            () -> startingPosition == StartingPosition.LEFT_BARGE,
                                            isRedAlliance,
                                            warmupCommands),
                                    AutoFactory.getPathfindToFeederGroundCommand(
                                            container.getDriveSubsystem(),
                                            container.getRobotState(),
                                            () -> startingPosition == StartingPosition.LEFT_BARGE,
                                            isRedAlliance,
                                            warmupCommands),
                                    () -> retrying)
                            .raceWith(
                                    new WaitUntilCommand(
                                                    () ->
                                                            container
                                                                            .getCoralStateTracker()
                                                                            .getCurrentPosition()
                                                                    != CoralPosition.STAGED_IN_CLAW)
                                            .andThen(
                                                    Commands.waitUntil(
                                                                    () ->
                                                                            container
                                                                                            .getCoralStateTracker()
                                                                                            .getCurrentPosition()
                                                                                    != CoralPosition
                                                                                            .NONE)
                                                            .deadlineFor(
                                                                    SuperstructureFactory
                                                                            .groundIntakeNoIndexerNoEnd(
                                                                                    container)
                                                                            .asProxy())));
        }
        return pathfindingCmd;
    }

    public Optional<Pose2d> getStartingPose() {
        return Optional.ofNullable(startingPose);
    }

    public Command getWarmupCommand() {
        ChezySequenceCommandGroup warmupCommand = new ChezySequenceCommandGroup();
        Pose2d startPose = startingPose;
        for (PathfindingAutoAlignCommand cmd : warmupCommands) {
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
