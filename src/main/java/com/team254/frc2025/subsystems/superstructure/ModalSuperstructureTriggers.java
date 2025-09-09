package com.team254.frc2025.subsystems.superstructure;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.team254.frc2025.subsystems.claw.ClawSubsystem;
import org.littletonrobotics.junction.Logger;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.factories.AutoFactory;
import com.team254.frc2025.factories.ClawFactory;
import com.team254.frc2025.factories.ClimberFactory;
import com.team254.frc2025.factories.IndexerFactory;
import com.team254.frc2025.factories.IntakeFactory;
import com.team254.frc2025.factories.LedFactory;
import com.team254.frc2025.factories.SuperstructureFactory;
import com.team254.frc2025.controlboard.ControlBoard;
import com.team254.frc2025.subsystems.led.LedState;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.lib.commands.ChezySequenceCommandGroup;
import com.team254.lib.reefscape.ReefBranch;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.FieldConstants;
import com.team254.lib.util.ReefscapeUtil;
import com.team254.lib.util.Util;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ModalSuperstructureTriggers {
    private final RobotContainer container;
    private final SuperstructureStateMachine stateMachine;
    private final CoralStateTracker coralTracker;
    private final AtomicReference<SuperstructureState> latestAlgaeStageState = new AtomicReference<>(
            SuperstructureState.STAGE_BARGE);
    private final AtomicReference<SuperstructureState> latestCoralStageState = new AtomicReference<>(
            SuperstructureState.STAGE_CORAL_L4);
    private final AtomicBoolean checkIfCoralAtStageBanner = new AtomicBoolean(false);
    private final AtomicReference<ReefBranch> selectedBranch = new AtomicReference<>(ReefBranch.AB);
    private final AtomicBoolean useFirstLetter = new AtomicBoolean(true);

    private AtomicBoolean enableAutoStow = new AtomicBoolean(false);

    private AtomicBoolean defaultRobotWide = new AtomicBoolean(true);

    private AtomicBoolean enableSnapHeadingCoral = new AtomicBoolean(true);
    private AtomicBoolean enableSnapHeadingAlgae = new AtomicBoolean(false);

    private AtomicBoolean uncancellableCommandRunning = new AtomicBoolean(false);

    public ModalSuperstructureTriggers(RobotContainer container, SuperstructureStateMachine stateMachine,
            CoralStateTracker tracker) {
        this.container = container;
        this.stateMachine = stateMachine;
        this.coralTracker = tracker;
        configureTriggers();
    }

    private Command setUncancellableCommandRunning(boolean set) {
        return new InstantCommand(() -> uncancellableCommandRunning.set(set));
    }

    private void setUncancellableCommandRunningNoCommand(boolean set) {
        uncancellableCommandRunning.set(set);
    }

    public void setCheckIfCoralAtStageBanner(boolean set) {
        checkIfCoralAtStageBanner.set(set);
    }

    public AtomicReference<SuperstructureState> getLatestCoralStageState() {
        return latestCoralStageState;
    }

    public AtomicReference<SuperstructureState> getLatestAlgaeStageState() {
        return latestAlgaeStageState;
    }

    public AtomicBoolean getDefaultRobotWide() {
        return defaultRobotWide;
    }

    public AtomicBoolean getEnableSnapHeadingCoral() {
        return enableSnapHeadingCoral;
    }

    public AtomicBoolean getEnableSnapHeadingAlgae() {
        return enableSnapHeadingAlgae;
    }

    private boolean isScoreableState(SuperstructureState state) {
        return state == SuperstructureState.STAGE_CORAL_L1 ||
                state == SuperstructureState.STAGE_CORAL_L2 ||
                state == SuperstructureState.STAGE_CORAL_L3 ||
                state == SuperstructureState.STAGE_CORAL_L4 ||
                state == SuperstructureState.STAGE_BARGE ||
                state == SuperstructureState.STAGE_PROCESSOR;
    }

    public boolean isScoreableStageState(SuperstructureState state) {
        return state == SuperstructureState.STAGE_CORAL_L2 ||
                state == SuperstructureState.STAGE_CORAL_L3 ||
                state == SuperstructureState.STAGE_CORAL_L4;
    }

    @SuppressWarnings("unused")
    private boolean isCoralStageState(SuperstructureState state) {
        return state == SuperstructureState.STAGE_CORAL_L2 ||
                state == SuperstructureState.STAGE_CORAL_L3 ||
                state == SuperstructureState.STAGE_CORAL_L4;
    }

    private boolean isNonDescorePosition(SuperstructureState state) {
        return state == SuperstructureState.NON_DESCORE_CORAL_L2 ||
                state == SuperstructureState.NON_DESCORE_CORAL_L3 ||
                state == SuperstructureState.NON_DESCORE_CORAL_L4 ||
                state == SuperstructureState.INTERMEDIATE_CORAL_L4;
    }

    @SuppressWarnings("unused")
    private boolean isAlgaeDescoreState(SuperstructureState state) {
        return state == SuperstructureState.STAGE_CORAL_L2 ||
                state == SuperstructureState.STAGE_CORAL_L3;
    }

    @SuppressWarnings("unused")
    private boolean isReefIntakeState(SuperstructureState state) {
        return state == SuperstructureState.REEF_ALGAE_INTAKE_L2 ||
                state == SuperstructureState.REEF_ALGAE_INTAKE_L3;
    }

    private boolean closeToCoralStation() {
        Translation2d rightCenterFace = FieldConstants.CoralStation.rightCenterFace.getTranslation();
        double distanceToRight = container.getRobotState().getLatestFieldToRobot().getValue().getTranslation()
                .getDistance(container.getRobotState().isRedAlliance()
                        ? Util.flipRedBlue(rightCenterFace)
                        : rightCenterFace);
        Translation2d leftCenterFace = FieldConstants.CoralStation.leftCenterFace.getTranslation();
        double distanceToLeft = container.getRobotState().getLatestFieldToRobot().getValue().getTranslation()
                .getDistance(container.getRobotState().isRedAlliance()
                        ? Util.flipRedBlue(leftCenterFace)
                        : leftCenterFace);
        boolean close = distanceToLeft < 4 || distanceToRight < 4;
        Logger.recordOutput("ReefscapeUtils/closeToCoral", close);
        return close;
    }

    private boolean shouldDisableAlgaeAutoStow() {
        if (container.getRobotState().onOpponentSide()) {
                return stateMachine.getClosestOpponentFace(container.getRobotState().isRedAlliance()) == ReefBranch.GH
                        || stateMachine.getClosestOpponentFace(container.getRobotState().isRedAlliance()) == ReefBranch.EF;
        }
        return stateMachine.getClosestFace(container.getRobotState().isRedAlliance()) == ReefBranch.GH
                || stateMachine.getClosestFace(container.getRobotState().isRedAlliance()) == ReefBranch.IJ;
    }

    private Command setStateCommand(SuperstructureState state, String name) {
        return new InstantCommand(() -> stateMachine.setDesiredState(state))
                .withName(name);
    }

    private Command setStateCommand(SuperstructureState state, boolean setFuture, String name) {
        return new InstantCommand(() -> stateMachine.setDesiredState(state, setFuture, true))
                .withName(name);
    }

    private Command setStateCommand(AtomicReference<SuperstructureState> state, boolean setFuture, String name) {
        return new InstantCommand(() -> stateMachine.setDesiredState(state.get(), setFuture, true))
                .withName(name);
    }

    public Command setStateAndWaitCommand(SuperstructureState state, String name) {
        return Commands.sequence(
                setStateCommand(state, name),
                new WaitUntilCommand(() -> container.getStateMachine().getCurrentState() != null
                        && stateMachine.getCurrentState().equals(state)));
    }

    public Command setFutureStateAndWaitUntilDesiredCommand(SuperstructureState state, String name) {
        return Commands.sequence(
                setStateCommand(state, true, name),
                new WaitUntilCommand(() -> (container.getStateMachine().getDesiredState() != null
                        && container.getStateMachine().getDesiredState().equals(state)) ||
                        (container.getStateMachine().getCurrentState() != null
                                && container.getStateMachine().getCurrentState().equals(state))));
    }

    public Command setFutureStateAndWaitUntilDesiredCommand(AtomicReference<SuperstructureState> state, String name) {
        return Commands.sequence(
                setStateCommand(state, true, name),
                new WaitUntilCommand(() -> (container.getStateMachine().getDesiredState() != null
                        && container.getStateMachine().getDesiredState().equals(state.get())) ||
                        (container.getStateMachine().getCurrentState() != null
                                && container.getStateMachine().getCurrentState().equals(state.get()))));
    }

    public Command overrideCurrentDesiredStateAndWaitUntilDesiredCommand(SuperstructureState currentState, AtomicReference<SuperstructureState> desiredState,
            String name) {
        return Commands.sequence(
                new InstantCommand(
                        () -> {
                            stateMachine
                                    .setDesiredState(desiredState.get(), true, true);
                        }),
                new WaitUntilCommand(() -> (container.getStateMachine().getDesiredState() != null
                        && container.getStateMachine().getDesiredState().equals(desiredState.get())) ||
                        (container.getStateMachine().getCurrentState() != null
                                && container.getStateMachine().getCurrentState().equals(desiredState.get()))));
    }

    private Command setAvoidDescoreCommand() {
        return Commands.defer(() -> {
            if (isScoreableStageState(stateMachine.getCurrentState())) {
                return new InstantCommand(() -> stateMachine.setDesiredState(
                        stateMachine.getCurrentState().getAvoidDescoreState()));
            }
            return Commands.none();
        }, Set.of());
    }

    public Command slapAlgaeCommand() {
        return Commands.defer(() -> {
            return setStateAndWaitCommand(SuperstructureState.REEF_ALGAE_SLAP, "Slap");
        }, Set.of());
    }

    public Command stageReefIntakeAlgaeCommand(ReefBranch branch) {
        return Commands.defer(() -> {
            if (branch == ReefBranch.AB || branch == ReefBranch.EF || branch == ReefBranch.IJ) {
                return setStateAndWaitCommand(SuperstructureState.REEF_ALGAE_INTAKE_L3, "Intake Algae L3");
            }
            return setStateAndWaitCommand(SuperstructureState.REEF_ALGAE_INTAKE_L2, "Intake Algae L2");
        }, Set.of());

    }

    public Command setFutureWaitDesiredReefIntakeAlgaeCommand(ReefBranch branch) {
        return Commands.defer(() -> {
            if (branch == ReefBranch.AB || branch == ReefBranch.EF || branch == ReefBranch.IJ) {
                return setFutureStateAndWaitUntilDesiredCommand(SuperstructureState.REEF_ALGAE_INTAKE_L3, "Intake Algae L3");
            }
            return setFutureStateAndWaitUntilDesiredCommand(SuperstructureState.REEF_ALGAE_INTAKE_L2, "Intake Algae L2");
        }, Set.of());
    }

    public SuperstructureState getCorrectReefIntakeState(ReefBranch branch) {
        if (branch == ReefBranch.AB || branch == ReefBranch.EF || branch == ReefBranch.IJ) {
            return SuperstructureState.REEF_ALGAE_INTAKE_L3;
        }
        return SuperstructureState.REEF_ALGAE_INTAKE_L2;
    }

    @SuppressWarnings("unused")
    private Command commandToCurrentStateCommand() {
        return Commands.defer(
                () -> setStateAndWaitCommand(stateMachine.getCurrentState(), "Stage Current State"),
                Set.of());
    }

    private Command commandToDesiredStateCommand() {
        return Commands.defer(
                () -> setStateAndWaitCommand(stateMachine.getDesiredState(), "Stage Desired State"),
                Set.of());
    }

    private Command createScoreCoralManualCommand() {
        Command scoreCommand = new InstantCommand(() -> container.getClaw().setCurrentPosition(-10))
                                                .andThen(Commands.waitSeconds(1));
        return new ConditionalCommand(
                scoreCommand,
                Commands.none(),
                () -> isScoreableState(stateMachine.getCurrentState()))
                .withName("Superstructure Coral Score");
    }

    private Command createScoreCoralCommand() {
        Command stageCommand = commandToDesiredStateCommand();
        Command scoreCommand = new InstantCommand(() -> container.getClaw().setCurrentPosition(-10))
                                                .andThen(Commands.waitSeconds(1));
        Command fullScoreSequence = Commands.sequence(stageCommand, scoreCommand);
        return new ConditionalCommand(
                fullScoreSequence,
                Commands.none(),
                () -> isScoreableState(stateMachine.getCurrentState()))
                .withName("Superstructure Coral Score");
    }

    private Command createCoralExhaustCommand() {
        return IntakeFactory.exhaustRollers(container).alongWith(IndexerFactory.exhaustIndexer(container))
                .alongWith(ClawFactory.exhaustCoral(container)
                        .onlyIf(() -> coralTracker.getCurrentPosition() != CoralPosition.STAGED_IN_CLAW))
                .withName("Coral Exhaust");
    }

    private Command createCoralExhaustManualCommand() {
        return IntakeFactory.exhaustRollers(container).alongWith(IndexerFactory.exhaustIndexer(container))
                .alongWith(ClawFactory.exhaustCoral(container))
                .withName("Coral Exhaust Manual");
    }

    private Command createHandoffFromSecondIndexerCommand() {
        Command handoff = new ChezySequenceCommandGroup(setUncancellableCommandRunning(true),
                IndexerFactory.runIndexer(container)
                        .withDeadline(ClawFactory.stageCoralInClaw(container, latestCoralStageState)
                                .until(() -> container.getCoralStateTracker()
                                        .getCurrentPosition() == CoralPosition.NONE
                                        || getManualOverrideButtons().getAsBoolean()))
                        .finallyDo(() -> setUncancellableCommandRunningNoCommand(false)));
        return handoff.withName("Handoff From Second Indexer");
    }

    private Command createHandoffFromFirstIndexerCommand() {
        Command handoff = new ChezySequenceCommandGroup(setUncancellableCommandRunning(true),
                IndexerFactory.runIndexer(container)
                        .withDeadline(ClawFactory.stageCoralInClaw(container, latestCoralStageState)
                                .until(() -> container.getCoralStateTracker()
                                        .getCurrentPosition() == CoralPosition.NONE
                                        || getManualOverrideButtons().getAsBoolean()
                                        || container.getModalControls().algaeClimbMode().getAsBoolean()))
                        .finallyDo(() -> setUncancellableCommandRunningNoCommand(false)));
        return handoff.withName("Handoff From First Indexer");
    }

    @SuppressWarnings("unused")
    private Command createHandoffToSecondIndexerCommand() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> container.getIndexer().setForwardLimit(true)),
                IndexerFactory.runIndexer(container)
                        .until(() -> getManualOverrideButtons().getAsBoolean()
                                || (container.getCoralStateTracker()
                                        .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                                        || container.getCoralStateTracker()
                                                .getCurrentPosition() == CoralPosition.NONE
                                        || container.getCoralStateTracker()
                                                .getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW
                                        || container.getCoralStateTracker()
                                                .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)))
                .finallyDo(() -> container.getIndexer().setForwardLimit(false))
                .withName("HandoffToSecondIndexer");
    }

    private Command createRunIntakeWhileHandoffCommand() {
        return IntakeFactory.spinRollers(container)
                .until(() -> getManualOverrideButtons().getAsBoolean()
                        || (container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                                || container.getCoralStateTracker()
                                        .getCurrentPosition() == CoralPosition.NONE
                                || container.getCoralStateTracker()
                                        .getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW
                                || container.getCoralStateTracker()
                                        .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)
                        || container.getModalControls().algaeClimbMode().getAsBoolean())
                .withName("RunIntakeWhileHandoff");
    }

    private Command createStageOrScoreCoralCommand() {
        return Commands.defer(() -> new ConditionalCommand(
                createScoreCoralCommand(),
                setStateCommand(latestCoralStageState.get(), "Stage Coral"),
                () -> stateMachine.getCurrentState().equals(latestCoralStageState.get()) && stateMachine.getDesiredState().equals(latestCoralStageState.get())),
                Set.of(container.getClaw()))
                .withName("Stage or Score Coral");
    }

    private Command createStageOrScoreCoralManualCommand() {
        return Commands.defer(() -> new ConditionalCommand(
                createScoreCoralManualCommand(),
                new InstantCommand(() -> {
                    stateMachine.setCurrentState(stateMachine.getDesiredState());
                    stateMachine.setTransitioning(false);
                })
                        .andThen(setStateCommand(latestCoralStageState.get(),
                                "Stage Coral Manual")),
                () -> stateMachine.getCurrentState().equals(latestCoralStageState.get()) && stateMachine.getDesiredState().equals(latestCoralStageState.get())),
                Set.of(container.getClaw()))
                .withName("Stage or Score Coral");
    }

    public Command createStageOrScoreAlgaeCommand() {
        return Commands.defer(() -> new ConditionalCommand(
                Commands.either(
                        ClawFactory.scoreAlgaeBarge(container)
                                .andThen(setStateCommand(SuperstructureState.STOW_CORAL,
                                        "Stow Coral")),
                        ClawFactory.scoreAlgaeProcessor(container)
                                .until(container.getModalControls().moveToAlgaeLevel()
                                        .negate()),
                        () -> stateMachine
                                .getCurrentState() == SuperstructureState.STAGE_BARGE),
                setStateCommand(latestAlgaeStageState.get(), "Stage Algae"),
                () -> (stateMachine.getCurrentState() == SuperstructureState.STAGE_BARGE
                        || stateMachine.getCurrentState() == SuperstructureState.STAGE_PROCESSOR)
                        && stateMachine.getCurrentState() == latestAlgaeStageState.get()),
                Set.of(container.getClaw()))
                .withName("Stage or Score Algae");
    }

    private Command createStowAlgaeCommand() {
        return new ConditionalCommand(
                Commands.either(
                        setStateCommand(SuperstructureState.STAGE_PROCESSOR, "Stow Algae"),
                        setStateCommand(SuperstructureState.STOW_ALGAE, "Stage Processor"),
                        () -> latestAlgaeStageState
                                .get() == SuperstructureState.STAGE_PROCESSOR),
                setStateCommand(SuperstructureState.STOW_CORAL, "Stow Coral"),
                () -> container.getClaw().hasAlgae());

    }

    public Trigger getManualOverrideButtons() {
        return container.getModalControls().coralExhaust().or(container.getModalControls().coralExhaustManual())
                .or(container.getModalControls().groundIntakeDeployManual())
                .or(container.getModalControls().groundIntakeDeploySpinManual());
    }

    private void configureTriggers() {
        Trigger stageCoralIfStageBannerTriggeredAtInit = new Trigger(
                () -> checkIfCoralAtStageBanner.get() && container.getClaw().hasCoralAtStageBanner()
                        && container.getStateMachine()
                                .getDesiredState() == SuperstructureState.STOW_CORAL);
        stageCoralIfStageBannerTriggeredAtInit.onTrue(Commands
                .waitUntil(() -> container.getStateMachine()
                        .getCurrentState() == SuperstructureState.STOW_CORAL)
                .andThen(container
                        .getClaw()
                        .dutyCycleCommand(
                                () -> Constants.ClawConstants.kCoralRollerDutyCycleRollback)
                        .withTimeout(0.5)
                        .andThen(
                                new InstantCommand(
                                        () -> {
                                            container.getCoralStateTracker()
                                                    .forceSet(CoralPosition.STAGED_IN_CLAW);
                                            checkIfCoralAtStageBanner
                                                    .set(false);
                                        })))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("Init_coral"));

        Trigger setL3L2Voltage = new Trigger(() -> stateMachine.getCurrentState() == SuperstructureState.STAGE_CORAL_L3
                || stateMachine.getCurrentState() == SuperstructureState.NON_DESCORE_CORAL_L3
                || stateMachine.getCurrentState() == SuperstructureState.NON_DESCORE_CORAL_L2
                || stateMachine.getCurrentState() == SuperstructureState.STAGE_CORAL_L2);
        setL3L2Voltage.onTrue(
                new InstantCommand(() -> container.getClaw().setWantedVoltageMode(ClawSubsystem.VoltageMode.L3_L2)).ignoringDisable(true));
        setL3L2Voltage.onFalse(
                new InstantCommand(() -> container.getClaw().setWantedVoltageMode(ClawSubsystem.VoltageMode.DEFAULT)).ignoringDisable(true));

        Trigger tryToPushCoralBackToIndexerState = new Trigger(
                () -> (container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.AT_FIRST_INDEXER
                        && !container.getIndexer().hasCoralAtFirstIndexerBanner()) ||
                        (container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                                && !container.getIndexer()
                                        .hasCoralAtSecondIndexerBanner()))
                .and(getManualOverrideButtons().negate());

        tryToPushCoralBackToIndexerState.whileTrue(IndexerFactory.runIndexer(container)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(2)
                .withName("PushCoralToIndexer"));

        Trigger tryToPushCoralBackToClawState = new Trigger(
                () -> container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW
                        && !container.getClaw().hasCoralAtStageBanner()
                        && (stateMachine.getCurrentState() == SuperstructureState.STOW_CORAL));
        tryToPushCoralBackToClawState.whileTrue(container
                .getClaw()
                .dutyCycleCommand(() -> Constants.ClawConstants.kCoralRollerDutyCycleRollback)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(2)
                .withName("PushCoralBackToClaw"));

        Trigger handoffFromSecondIndexerBannerToClaw = new Trigger(
                () -> (container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER)
                        && !(container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)
                        && (stateMachine.getCurrentState() == SuperstructureState.STOW_CORAL) && !getManualOverrideButtons().getAsBoolean());
        handoffFromSecondIndexerBannerToClaw.onTrue(
                createHandoffFromSecondIndexerCommand()
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .ignoringDisable(true));

        Trigger handoffToSecondIndexerBanner = new Trigger(
                () -> (container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.AT_FIRST_INDEXER)
                        && (stateMachine.getCurrentState() != SuperstructureState.STOW_CORAL)
                        && !getManualOverrideButtons().getAsBoolean());
        handoffToSecondIndexerBanner.onTrue(container.getIndexer().dutyCycleCommand(() -> 0)
                .withName("Keep coral at first banner"));

        Trigger handoffFromFirstIndexerBannerToClaw = new Trigger(
                () -> (container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.AT_FIRST_INDEXER)
                        && (stateMachine.getCurrentState() == SuperstructureState.STOW_CORAL)
                        && !container.getModalControls().algaeClimbMode().getAsBoolean()
                        && !getManualOverrideButtons().getAsBoolean());
        handoffFromFirstIndexerBannerToClaw.onTrue(
                createHandoffFromFirstIndexerCommand()
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .ignoringDisable(true));

        handoffFromFirstIndexerBannerToClaw.onTrue(
                createRunIntakeWhileHandoffCommand()
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                        .ignoringDisable(true));

        Trigger stowIntakeAtSecondBanner = new Trigger(
                () -> ((container.getCoralStateTracker().getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                        || container.getCoralStateTracker().getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW)
                        && !container.getIndexer().hasCoralAtFirstIndexerBanner()
                        && !getManualOverrideButtons().getAsBoolean() && !DriverStation.isAutonomous()));
        stowIntakeAtSecondBanner.onTrue(
            setUncancellableCommandRunning(true).andThen(Commands.parallel(
                        IntakeFactory.exhaustRollers(container).withTimeout(1),
                        Commands.either(
                                IntakeFactory.deployIntakeBlocking(container),
                                IntakeFactory.stowIntakeBlocking(container),
                                () -> defaultRobotWide.get()).withTimeout(1)))
                        .until(() -> getManualOverrideButtons().getAsBoolean())
                        .finallyDo(() -> setUncancellableCommandRunningNoCommand(false))
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withName("Stow Intake At Second Banner"));

        Trigger exhaustSecondCoral = new Trigger(
                () -> ((container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW)
                        && (container.getIndexer().hasCoralAtFirstIndexerBanner() && !getManualOverrideButtons().getAsBoolean() && !DriverStation.isAutonomous())));
        exhaustSecondCoral
                .onTrue(setUncancellableCommandRunning(true).andThen(IntakeFactory.exhaustRollers(container)
                        .alongWith(IndexerFactory.exhaustIndexer(container))
                        .until(() -> (!container.getIndexer().hasCoralAtFirstIndexerBanner()
                                && !container.getIndexer()
                                        .hasCoralAtSecondIndexerBanner()))
                        .andThen(Commands.waitSeconds(2))
                        .until(() -> getManualOverrideButtons().getAsBoolean())
                        .finallyDo(() -> setUncancellableCommandRunningNoCommand(false)))
                        .ignoringDisable(true)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withName("Exhaust Second Coral"));

        Trigger runIndexerCloseToCoralStation = new Trigger(
                () -> !(container.getCoralStateTracker()
                        .getCurrentPosition() == CoralPosition.AT_FIRST_INDEXER)
                        && !(container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.GOING_TO_SECOND_INDEXER)
                        &&
                        !(container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER)
                        &&
                        !(container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)
                        &&
                        closeToCoralStation());
        runIndexerCloseToCoralStation
                .and(() -> DriverStation.isAutonomous())
                .and(container.getModalControls().groundCoralIntake().negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .whileTrue(IndexerFactory.runIndexer(container)
                        .withName("run indexer close to feeder"));

        // auto stow with canrange sensors
        Trigger enableAutoStowTrigger = new Trigger(
                () -> !container.getClaw().hasCoralAtStageBanner()
                        && !container.getClaw().hasCoralAtScoreBanner()
                        && enableAutoStow.get() &&
                        (DriverStation.isAutonomous() || container.getModalControls()
                                .coralMode().getAsBoolean()));

        enableAutoStowTrigger.onTrue(
                Commands.waitUntil(() -> (container.getElevator().canStowElevator()
                        && ReefscapeUtil.awayFromReef(container,
                                0.4))
                        || stateMachine.getCurrentState() == SuperstructureState.STAGE_CORAL_L1)
                        .andThen(
                                new InstantCommand(() -> enableAutoStow.set(false))
                                        .andThen(
                                                new InstantCommand(
                                                        () -> Logger.recordOutput(
                                                                "Last Unstow Time",
                                                                Timer.getFPGATimestamp())),
                                                Commands.either(
                                                    Commands.either(setStateCommand(SuperstructureState.STOW_CORAL, "Auto Stow after L1"), new InstantCommand(
                                                        () -> {
                                                            stateMachine
                                                                    .setDesiredState(
                                                                            SuperstructureState.STOW_ELEVATOR, true,
                                                                            false);
                                                        })
                                                        .alongWith(Commands
                                                                .waitUntil(() -> (container
                                                                        .getElevator()
                                                                        .canStow()
                                                                        && ReefscapeUtil.awayFromReef(
                                                                                container))
                                                                        || stateMachine
                                                                                .getCurrentState() == SuperstructureState.STAGE_CORAL_L1)
                                                                .andThen(new InstantCommand(
                                                                        () -> {
                                                                            stateMachine.setTransitioning(false);
                                                                            stateMachine.setCurrentState(
                                                                                    SuperstructureState.STOW_ELEVATOR);
                                                                            stateMachine
                                                                                    .setDesiredState(
                                                                                            SuperstructureState.STOW_CORAL,
                                                                                            true, false);
                                                                        }))), () -> stateMachine.getCurrentState() == SuperstructureState.STAGE_CORAL_L1),
                                                        new ChezySequenceCommandGroup(Commands
                                                                .waitUntil(() -> (container
                                                                        .getElevator()
                                                                        .canStow()
                                                                        && ReefscapeUtil.awayFromReef(
                                                                                container))
                                                                        || stateMachine
                                                                                .getCurrentState() == SuperstructureState.STAGE_CORAL_L1),
                                                                Commands.defer(() -> stageReefIntakeAlgaeCommand(
                                                                        stateMachine
                                                                                .getClosestFace(container
                                                                                        .getRobotState()
                                                                                        .isRedAlliance())),
                                                                        Set.of())).onlyIf(() -> !DriverStation.isAutonomous()),
                                                        container.getModalControls()
                                                                .algaeClimbMode()
                                                                .negate())))
                        .withTimeout(3.0)
                        .withName("AutoStow"));

        Trigger enableAutoStowAlgaeTrigger = new Trigger(
                () -> container.getClaw().hasAlgae()
                        && (stateMachine.getCurrentState() == SuperstructureState.REEF_ALGAE_INTAKE_L2
                                || stateMachine.getCurrentState() == SuperstructureState.REEF_ALGAE_INTAKE_L3)
                        && stateMachine.getCurrentState() == stateMachine.getDesiredState()
                        && container.getModalControls().algaeClimbMode().getAsBoolean()
                        && (!shouldDisableAlgaeAutoStow() || latestAlgaeStageState.get() == SuperstructureState.STAGE_PROCESSOR)
                        && !DriverStation.isAutonomous());

        enableAutoStowAlgaeTrigger.onTrue(
                Commands.sequence(
                        Commands.waitUntil(
                                () -> container.getElevator().canStow() && ReefscapeUtil.awayFromReef(container)),
                        new InstantCommand(() -> Logger.recordOutput("Latest Autostow Algae Start",
                                RobotTime.getTimestampSeconds())),
                        // check that all initial conditions are still met
                        createStowAlgaeCommand()
                                .onlyIf(enableAutoStowAlgaeTrigger
                                        .and(container.getModalControls().coralMode().negate())))
                        .withTimeout(3.0).withName("Auto Stow Algae"));

        Trigger avoidDescoreAfterScore = new Trigger(
                () -> container.getClaw().getPositionRotations() < -5 &&
                        stateMachine.getCurrentState() == stateMachine.getDesiredState() &&
                        isScoreableStageState(stateMachine.getDesiredState())
                        && container.getClaw().autoScoreEnabled());
        avoidDescoreAfterScore.onTrue(setAvoidDescoreCommand()
                .andThen(new InstantCommand(
                        () -> container.getCoralStateTracker().forceSet(CoralPosition.NONE))));
        Trigger setAutoStowTrigger = new Trigger(() -> isNonDescorePosition(stateMachine.getCurrentState())
                || isNonDescorePosition(stateMachine.getDesiredState()));
        setAutoStowTrigger.onTrue(Commands.runOnce(() -> {
            enableAutoStow.set(true);
        }));

        // also enable autostow on L1 ONLY in CORAL mode (avoids running this in the
        // pit)
        Trigger L1AutoStowTrigger = new Trigger(
                () -> stateMachine.getCurrentState() == SuperstructureState.STAGE_CORAL_L1
                        && !container.getClaw().hasCoralAtStageBanner());
        container.getModalControls().coralMode().and(L1AutoStowTrigger).onTrue(
                Commands.runOnce(() -> {
                    enableAutoStow.set(true);
                }));

        Trigger autoReefIntakeAlgae = new Trigger(
                () -> (stateMachine.getDesiredState() == SuperstructureState.REEF_ALGAE_INTAKE_L2
                        || stateMachine.getDesiredState() == SuperstructureState.REEF_ALGAE_SLAP
                        || stateMachine.getDesiredState() == SuperstructureState.REEF_ALGAE_INTAKE_L3
                        || stateMachine.getDesiredState() == SuperstructureState.REEF_ALGAE_INTAKE_L3_MANUAL)
                        && !container.getClaw().hasAlgae()
                        && container.getModalControls().algaeClimbMode().getAsBoolean());

        autoReefIntakeAlgae.debounce(0.25, DebounceType.kFalling).whileTrue(ClawFactory.intakeAlgae(container));

        Trigger disableAutoScore = new Trigger(() -> !isScoreableStageState(stateMachine.getDesiredState()) && container.getClaw().autoScoreEnabled());
        Trigger enableAutoScore = new Trigger(() -> isScoreableStageState(stateMachine.getCurrentState())
                                        && stateMachine.getCurrentState() == stateMachine.getDesiredState()
                                        && !container.getClaw().autoScoreEnabled());

        disableAutoScore.onTrue(new InstantCommand(() -> container.getClaw().setAutoScore(false))
                .alongWith(new InstantCommand(() -> Logger.recordOutput(
                        "Last_Disable_AutoScore_Timestamp", RobotTime.getTimestampSeconds()))).ignoringDisable(true));
        enableAutoScore.onTrue(
                new ChezySequenceCommandGroup(
                        Commands.parallel(
                                new InstantCommand(() -> container.getClaw().setCurrentPosition(0.0)),
                                new InstantCommand(() -> Logger.recordOutput("Last_Position_Reset_Timestamp", RobotTime.getTimestampSeconds()))),
                        Commands.parallel(
                                new InstantCommand(() -> container.getClaw().setAutoScore(true)),
                                new InstantCommand(() -> Logger.recordOutput("Last_Enable_AutoScore_Timestamp", RobotTime.getTimestampSeconds())))
                ).ignoringDisable(true));


        container.getModalControls().setDefaultRobotWide()
                .and(container.getModalControls().coralManualMode().negate()).onTrue(
                        Commands.parallel(
                                new InstantCommand(() -> defaultRobotWide.set(true)),
                                IntakeFactory.deployIntakeBlocking(container)));

        container.getModalControls().setDefaultRobotTight()
                .and(container.getModalControls().coralManualMode().negate()).onTrue(
                        Commands.parallel(
                                new InstantCommand(() -> defaultRobotWide.set(false)),
                                IntakeFactory.stowIntakeBlocking(container)
                                        .onlyIf(() -> !container.getIndexer()
                                                .hasCoralAtFirstIndexerBanner())));

        ////////////////////////////// CORAL MODE //////////////////////////////
        container.getModalControls().coralMode().onTrue(
                Commands.either(ClawFactory.exhaustAlgae(container),
                        Commands.none(),
                        () -> container.getClaw().hasAlgae())
                        .andThen(Commands.either(
                                new InstantCommand(
                                        () -> stateMachine.setDesiredState(
                                                SuperstructureState.STOW_CORAL)),
                                Commands.none(),
                                () -> ((ReefscapeUtil.awayFromReef(container)
                                        || stateMachine.getCurrentState() == SuperstructureState.STAGE_BARGE)
                                        && container.getElevator().canStow())
                                        || stateMachine.getCurrentState() == SuperstructureState.STAGE_PROCESSOR)));

        container.getModalControls().groundCoralIntake().and(handoffToSecondIndexerBanner.negate()).whileTrue(
                SuperstructureFactory.groundIntake(container).withName("Ground Intake"));
        container.getModalControls().groundCoralIntake()
                .and(() -> coralTracker.getCurrentPosition() == CoralPosition.STAGED_IN_CLAW
                        || coralTracker.getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                        || coralTracker.getCurrentPosition() == CoralPosition.GOING_TO_SECOND_INDEXER
                        || coralTracker.getCurrentPosition() == CoralPosition.AT_FIRST_INDEXER
                        || coralTracker.getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW)
                .whileTrue(
                        ControlBoard.getInstance().rumble().withName(
                                "Rumbling Controller for staging Coral Mode"));
        container.getModalControls().groundCoralIntake().and(getManualOverrideButtons().negate()).onFalse(
                IntakeFactory.spinRollers(container)
                        .alongWith(IndexerFactory.runIndexer(container))
                        .until(() -> coralTracker.getCurrentPosition() != CoralPosition.NONE)
                        .withTimeout(1)
                        .onlyIf(container.getModalControls().leftBranchSelect().or(container.getModalControls().rightBranchSelect()).and(() -> coralTracker.getCurrentPosition() == CoralPosition.NONE))
                .andThen(
        Commands.either(
                IntakeFactory.stowIntakeBlocking(container), Commands.none(),
                () -> (container.getCoralStateTracker().getCurrentPosition() == CoralPosition.NONE
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)
                        && !defaultRobotWide.get())).withName("Post Intaking"));
        container.getModalControls().stowCoral()
                .and(() -> container.getElevator().canStow() && ReefscapeUtil.awayFromReef(container))
                .onTrue(Commands.either(
                        setStateCommand(SuperstructureState.CLEAR_LIMELIGHT, "Clear Limelight"),
                        setStateCommand(SuperstructureState.STOW_CORAL, "Stow Coral"),
                        () -> coralTracker.getCurrentPosition() == CoralPosition.STAGED_IN_CLAW));

        container.getModalControls().leftBranchSelect()
                .and(() -> stateMachine.getCurrentState() != SuperstructureState.STAGE_CORAL_L1
                        || latestCoralStageState.get() != SuperstructureState.STAGE_CORAL_L1)
                .whileTrue(Commands.runOnce(() -> {
                    useFirstLetter.set(true);
                }).andThen(Commands.defer(() -> Commands.runOnce(() -> {
                    selectedBranch.set(
                            stateMachine.getClosestFace(container.getRobotState().isRedAlliance()));
                }), Set.of()))
                        .andThen(AutoFactory.getPathfindToReefCommand(container.getDriveSubsystem(),
                                container.getRobotState(),
                                selectedBranch::get,
                                useFirstLetter::get,
                                () -> container.getStateMachine().getDesiredState()
                                        .getElevatorHeightMeters(),
                                () -> overrideCurrentDesiredStateAndWaitUntilDesiredCommand(SuperstructureState.CLEAR_LIMELIGHT,
                                        latestCoralStageState, "Stage Coral"),
                                () -> stateMachine.getCurrentState() == latestCoralStageState
                                        .get(),
                                () -> container.getRobotState().isRedAlliance(), false, false))
                        .withName("AutoAlignLeft"))

                .onFalse(Commands.runOnce(() -> stateMachine.wipeFutureDesiredState()));
        container.getModalControls().rightBranchSelect()
                .and(() -> latestCoralStageState.get() != SuperstructureState.STAGE_CORAL_L1)
                .whileTrue(Commands.runOnce(() -> {
                    useFirstLetter.set(false);
                }).andThen(Commands.defer(() -> Commands.runOnce(() -> {
                    selectedBranch.set(stateMachine.getClosestFace(container.getRobotState().isRedAlliance()));
                }), Set.of()))
                        .andThen(AutoFactory.getPathfindToReefCommand(container.getDriveSubsystem(),
                                container.getRobotState(),
                                selectedBranch::get,
                                useFirstLetter::get,
                                () -> container.getStateMachine().getDesiredState().getElevatorHeightMeters(),
                                () -> overrideCurrentDesiredStateAndWaitUntilDesiredCommand(
                                        SuperstructureState.CLEAR_LIMELIGHT,
                                        latestCoralStageState, "Stage Coral"),
                                () -> stateMachine.getCurrentState() == latestCoralStageState.get(),
                                () -> container.getRobotState().isRedAlliance(), false, false))
                        .withName("AutoAlignRight"))
                .onFalse(Commands.runOnce(() -> stateMachine.wipeFutureDesiredState()));

        container.getModalControls().rightBranchSelect()
                .and(() -> latestCoralStageState.get() == SuperstructureState.STAGE_CORAL_L1)
                .onTrue(new ConditionalCommand(
                        Commands.waitUntil(() -> container.getStateMachine()
                                .getCurrentState() == SuperstructureState.STAGE_CORAL_L1)
                                .andThen(Commands
                                        .defer(() -> ClawFactory.scoreL1Deep(container), Set.of(container.getClaw()))
                                        .asProxy().withTimeout(0.5))
                                .withTimeout(2)
                                .withName("Score L1 Deep"),
                        Commands.defer(() -> setStateCommand(latestCoralStageState.get(), "Stage Coral"), Set.of()) ,
                        () -> (container.getCoralStateTracker().getCurrentPosition() != CoralPosition.STAGED_IN_CLAW
                                && container.getCoralStateTracker()
                                        .getCurrentPosition() != CoralPosition.PROCESSING_IN_CLAW)
                                || stateMachine.getDesiredState().equals(latestCoralStageState.get())));

        container.getModalControls().autoHP().onTrue(Commands.runOnce(() -> enableSnapHeadingCoral.set(false)));
        container.getModalControls().autoHP().onFalse(Commands.runOnce(() -> enableSnapHeadingCoral.set(true)));

        container.getModalControls().manualIntakeAlgae().onTrue(Commands.runOnce(() -> enableSnapHeadingAlgae.set(true)));
        container.getModalControls().manualIntakeAlgae().onFalse(Commands.runOnce(() -> enableSnapHeadingAlgae.set(false)));

        container.getModalControls().stageL1().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L1)));
        container.getModalControls().stageL2().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L2)));
        container.getModalControls().stageL3().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L3)));
        container.getModalControls().stageL4().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L4)));
        container.getModalControls().stageOrScoreCoral().and(() -> latestCoralStageState.get() != SuperstructureState.STAGE_CORAL_L1)
                .onTrue(createStageOrScoreCoralCommand());
        container.getModalControls().stageOrScoreCoral()
                .and(() -> latestCoralStageState.get() == SuperstructureState.STAGE_CORAL_L1)
                .onTrue(new ConditionalCommand(
                        Commands.waitUntil(() -> container.getStateMachine()
                                .getCurrentState() == SuperstructureState.STAGE_CORAL_L1)
                                .andThen(Commands
                                        .defer(() -> ClawFactory.scoreL1Shallow(container), Set.of(container.getClaw()))
                                        .asProxy().withTimeout(0.5))
                                .withTimeout(2)
                                .withName("Score L1 Shallow"),
                        Commands.defer(() -> setStateCommand(latestCoralStageState.get(), "Stage Coral"), Set.of()) ,
                        () -> (container.getCoralStateTracker().getCurrentPosition() != CoralPosition.STAGED_IN_CLAW
                                && container.getCoralStateTracker()
                                        .getCurrentPosition() != CoralPosition.PROCESSING_IN_CLAW)
                                || stateMachine.getDesiredState().equals(latestCoralStageState.get())));

        container.getModalControls().funnelIntakeCoral()
                .and(container.getModalControls().groundCoralIntake().negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .and(() -> (container.getElevator().canStow() && ReefscapeUtil.awayFromReef(container))
                        || stateMachine.getCurrentState() == SuperstructureState.STOW_CORAL)
                .whileTrue(
                        setStateAndWaitCommand(SuperstructureState.STOW_CORAL,
                                "Intake Coral Close To Station")
                                .andThen(IndexerFactory.runIndexer(container))
                                .withName("Superstructure Funnel Intake"));

        container.getModalControls().coralExhaust().and(() -> !uncancellableCommandRunning.get()).whileTrue(createCoralExhaustCommand());

        ////////////////////////////// ALGAE CLIMB MODE //////////////////////////////
        container.getModalControls().algaeClimbMode().onTrue(
                Commands.either(
                        createStowAlgaeCommand(),
                        Commands.none(),
                        () -> container.getElevator().canStow()
                                && ReefscapeUtil.awayFromReef(container)));

        Trigger shouldIntakeCoralinAlgaeMode = new Trigger(
                () -> container.getClaw().hasAlgae()
                        && stateMachine.getDesiredState() != SuperstructureState.GROUND_ALGAE_INTAKE
                        && stateMachine.getDesiredState() != SuperstructureState.LOLLIPOP_INTAKE);

        // ground/lollipop intake algae
        container.getModalControls().groundIntakeAlgae().and(shouldIntakeCoralinAlgaeMode.negate()).onTrue(
                setStateCommand(SuperstructureState.GROUND_ALGAE_INTAKE, "Ground Algae Intake"));
        container.getModalControls().lollipopIntake().and(shouldIntakeCoralinAlgaeMode.negate()).onTrue(
                setStateCommand(SuperstructureState.LOLLIPOP_INTAKE, "Lollipop Intake"));
        container.getModalControls().groundIntakeAlgae()
                .and(shouldIntakeCoralinAlgaeMode.negate()).debounce(0.25, DebounceType.kFalling)
                .whileTrue(ClawFactory.intakeAlgae(container));
        container.getModalControls().lollipopIntake()
                .and(shouldIntakeCoralinAlgaeMode.negate()).debounce(0.25, DebounceType.kFalling)
                .whileTrue(ClawFactory.intakeAlgae(container));

        // intake coral in algae mode
        container.getModalControls().groundIntakeAlgae().and(shouldIntakeCoralinAlgaeMode)
                .and(handoffToSecondIndexerBanner.negate()).whileTrue(
                        SuperstructureFactory.groundIntake(container));

        container.getModalControls().groundIntakeAlgae().or(container.getModalControls().lollipopIntake()).onFalse(
                Commands.waitSeconds(Constants.ClawConstants.kAlgaeRollerDebounceTime).onlyIf(shouldIntakeCoralinAlgaeMode.negate()).andThen(
                        new ParallelCommandGroup(
                                Commands.either(createStowAlgaeCommand(),
                                        Commands.none(),
                                        () -> stateMachine.getDesiredState() == SuperstructureState.GROUND_ALGAE_INTAKE
                                                || stateMachine.getDesiredState() == SuperstructureState.LOLLIPOP_INTAKE),
                                Commands.either(
                                        IntakeFactory.stowIntakeBlocking(
                                                container),
                                        Commands.none(),
                                        () -> (container.getCoralStateTracker()
                                                .getCurrentPosition() == CoralPosition.NONE
                                                || container.getCoralStateTracker()
                                                        .getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW
                                                || container.getCoralStateTracker()
                                                        .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                                                || container.getCoralStateTracker()
                                                        .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)
                                                && !defaultRobotWide
                                                        .get()))));

        container.getModalControls().stageAlgaeL2().onTrue(
                new InstantCommand(() -> latestAlgaeStageState
                        .set(SuperstructureState.REEF_ALGAE_INTAKE_L2)));
        container.getModalControls().stageAlgaeL3().onTrue(
                new InstantCommand(() -> latestAlgaeStageState
                        .set(SuperstructureState.REEF_ALGAE_INTAKE_L3_MANUAL)));
        container.getModalControls().bargeManualStage().onTrue(
                new InstantCommand(() -> latestAlgaeStageState.set(SuperstructureState.STAGE_BARGE)));
        container.getModalControls().processorManualStage().onTrue(
                new InstantCommand(
                        () -> latestAlgaeStageState.set(SuperstructureState.STAGE_PROCESSOR)));

        container.getModalControls().autoAlignReefIntake().whileTrue(Commands.runOnce(() -> {
            useFirstLetter.set(true);
        }).andThen(Commands.defer(() -> Commands.runOnce(() -> {
            selectedBranch.set(container.getRobotState().onOpponentSide() ? stateMachine.getClosestOpponentFace(container.getRobotState().isRedAlliance()) : stateMachine.getClosestFace(container.getRobotState().isRedAlliance()));
        }), Set.of()))
                .andThen(AutoFactory.getPathfindToAlgaeIntake(container, selectedBranch::get,
                        useFirstLetter::get, false))
                .withName("AlgaeReefAlign"));

        container.getModalControls().moveToAlgaeLevel().onTrue(
                createStageOrScoreAlgaeCommand());

        container.getModalControls().stowAlgae().and(
                () -> ((ReefscapeUtil.awayFromReef(container)
                        || stateMachine.getCurrentState() == SuperstructureState.STAGE_BARGE)
                        && container.getElevator().canStow())
                        || stateMachine.getCurrentState() == SuperstructureState.STAGE_PROCESSOR)
                .onTrue(createStowAlgaeCommand());
        container.getModalControls().deployClimber().debounce(0.25).onTrue(ClimberFactory.autoClimb(container).withName("Auto Climb"));
        container.getModalControls().deployClimber().debounce(0.25).onTrue(IntakeFactory.stowIntakeForClimbBlocking(container).withTimeout(2.0).withName("Stow Intake Before Climb"));
        container.getModalControls().deployClimber().debounce(0.25).onTrue(setStateCommand(SuperstructureState.STOW_CORAL, "Stow Coral Before Climb"));

        container.getModalControls().climb().whileTrue(
                ClimberFactory.manualJogClimber(container).withName("Manual Jog Climb"));

        Trigger groundAlgaeRumble = container.getModalControls().groundIntakeAlgae()
                .and(() -> container.getClaw().hasAlgae())
                .and(shouldIntakeCoralinAlgaeMode.negate());

        Trigger lollipopAlgaeRumble = container.getModalControls().lollipopIntake()
                .and(() -> container.getClaw().hasAlgae())
                .and(shouldIntakeCoralinAlgaeMode.negate());

        Trigger reefAlgaeRumble = container.getModalControls().autoAlignReefIntake()
                .and(() -> container.getClaw().hasAlgae());

        groundAlgaeRumble.or(reefAlgaeRumble).or(lollipopAlgaeRumble).whileTrue(
                container.getControlBoard().rumble().withName("Rumbling Controller for has algae"));

        Trigger hasCoralInAlgaeModeRumble = container.getModalControls().groundIntakeAlgae()
                .and(shouldIntakeCoralinAlgaeMode)
                .and(() -> coralTracker.getCurrentPosition() != CoralPosition.NONE);

        hasCoralInAlgaeModeRumble.whileTrue(
                container.getControlBoard().rumble().withName("Rumbling Controller for has coral in algae mode"));

        ////////////////////////////// CORAL MANUAL MODE //////////////////////////////
        container.getModalControls().groundIntakeDeployManual()
                .whileTrue(IntakeFactory.deployIntakeBlocking(container));
        container.getModalControls().groundIntakeDeploySpinManual().whileTrue(
                IntakeFactory.deployIntakeBlocking(container)
                        .alongWith(IntakeFactory.spinRollers(container)));
        container.getModalControls().groundCoralIntakeManual().and(handoffToSecondIndexerBanner.negate())
                .whileTrue(
                        SuperstructureFactory.groundIntake(container)
                                .withName("Ground Intake Manual"));
        container.getModalControls().groundCoralIntakeManual()
                .and(() -> coralTracker.getCurrentPosition() == CoralPosition.STAGED_IN_CLAW
                        || coralTracker.getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                        || coralTracker.getCurrentPosition() == CoralPosition.GOING_TO_SECOND_INDEXER
                        || coralTracker.getCurrentPosition() == CoralPosition.AT_FIRST_INDEXER
                        || coralTracker.getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW)
                .whileTrue(
                        ControlBoard.getInstance().rumble()
                                .withName("Rumbling Controller for staging Coral Manual Mode"));
        container.getModalControls().groundCoralIntakeManual().onFalse(Commands.either(
                IntakeFactory.stowIntakeBlocking(container), Commands.none(),
                () -> (container.getCoralStateTracker().getCurrentPosition() == CoralPosition.NONE
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.PROCESSING_IN_CLAW
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.AT_SECOND_INDEXER
                        || container.getCoralStateTracker()
                                .getCurrentPosition() == CoralPosition.STAGED_IN_CLAW)
                        && !defaultRobotWide.get()));

        container.getModalControls().stageL1Manual().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L1)));
        container.getModalControls().stageL2Manual().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L2)));
        container.getModalControls().stageL3Manual().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L3)));
        container.getModalControls().stageL4Manual().onTrue(
                new InstantCommand(
                        () -> latestCoralStageState.set(SuperstructureState.STAGE_CORAL_L4)));
        container.getModalControls().stageOrScoreCoralManual()
                .and(() -> latestCoralStageState.get() != SuperstructureState.STAGE_CORAL_L1)
                .onTrue(createStageOrScoreCoralManualCommand());
        container.getModalControls().stageOrScoreCoralManual()
                .and(() -> latestCoralStageState.get() == SuperstructureState.STAGE_CORAL_L1)
                .onTrue(new ConditionalCommand(
                        Commands.waitUntil(() -> container.getStateMachine()
                                .getCurrentState() == SuperstructureState.STAGE_CORAL_L1)
                                .andThen(Commands
                                        .defer(() -> ClawFactory.scoreL1Deep(container), Set.of(container.getClaw()))
                                        .asProxy().withTimeout(0.5))
                                .withTimeout(2)
                                .withName("Score L1 Deep"),
                        Commands.defer(() -> setStateCommand(latestCoralStageState.get(), "Stage Coral Manual"), Set.of()),
                        () -> (container.getCoralStateTracker().getCurrentPosition() != CoralPosition.STAGED_IN_CLAW
                                && container.getCoralStateTracker()
                                        .getCurrentPosition() != CoralPosition.PROCESSING_IN_CLAW)
                                || stateMachine.getDesiredState().equals(latestCoralStageState.get())));

        container.getModalControls().descoreManual()
                .onTrue(setAvoidDescoreCommand());

        container.getModalControls().stowCoralManual().onTrue(
                setStateCommand(SuperstructureState.STOW_CORAL, "Stow Coral Manual"));

        container.getModalControls().funnelIntakeCoralManual()
                .and(container.getModalControls().groundCoralIntake().negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .whileTrue(setStateAndWaitCommand(SuperstructureState.STOW_CORAL,
                        "Intake Coral Close To Station")
                        .andThen(IndexerFactory.runIndexer(container))
                        .withName("Superstructure Funnel Intake Manual"));

        container.getModalControls().coralExhaustManual().and(() -> !uncancellableCommandRunning.get())
                .whileTrue(createCoralExhaustManualCommand().withName("Superstructure Exhaust Manual"));

        ////////////////////////////// LEDs //////////////////////////////
        Trigger climberLatched = new Trigger(
                () -> container.getClimberRoller().climberLeftLimitSwitchTriggered()
                        && container.getClimberRoller().climberRightLimitSwitchTriggered());

        climberLatched
                .onTrue(LedFactory.latchedLEDs(container)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withName("Climber Latched LEDs"));

        Trigger notifyLowOnTime = new Trigger(() -> {
            // when in teleop tether mode the getMatchTime counts up. only do this when
            // counting down
            boolean isTeleopOnField = DriverStation.isTeleopEnabled() && DriverStation.isFMSAttached();
            double matchTime = DriverStation.getMatchTime();
            double ts = Timer.getFPGATimestamp();

            // teleop
            return isTeleopOnField &&
            // 25-35s
                    (matchTime >= 15.0 && matchTime <= 25.0) &&
            // for a small amount of time each second
                    ((ts - Math.floor(ts)) > 0.700);
        });

        container.getModalControls().coralMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L1)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntake().negate()).whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L1, () -> false)
                                .withName("Coral Mode LEDs L1"));

        container.getModalControls().coralMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L2)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntake().negate()).whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L2, () -> false)
                                .withName("Coral Mode LEDs L2"));

        container.getModalControls().coralMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L3)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntake().negate()).whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L3, () -> false)
                                .withName("Coral Mode LEDs L3"));

        container.getModalControls().coralMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L4)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntake().negate()).whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L4, () -> false)
                                .withName("Coral Mode LEDs L4"));
        container.getModalControls().groundCoralIntake().whileTrue(
                LedFactory.groundIntakeLEDs(container).withName("Ground Intake LEDs"));

        container.getModalControls().coralManualMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L1)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L1, () -> true)
                                .withName("Coral Manual Mode LEDs L1"));

        container.getModalControls().coralManualMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L2)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L2, () -> true)
                                .withName("Coral Manual Mode LEDs L2"));

        container.getModalControls().coralManualMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L3)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L3, () -> true)
                                .withName("Coral Manual Mode LEDs L3"));

        container.getModalControls().coralManualMode()
                .and(() -> getLatestCoralStageState().get() == SuperstructureState.STAGE_CORAL_L4)
                .and(notifyLowOnTime.negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .whileTrue(
                        LedFactory.coralStagingLEDs(container,
                                () -> SuperstructureState.STAGE_CORAL_L4, () -> true)
                                .withName("Coral Manual Mode LEDs L4"));

        container.getModalControls().groundCoralIntakeManual().whileTrue(
                LedFactory.groundIntakeLEDs(container).withName("Ground Intake LEDs"));

        Trigger climberLEDs = new Trigger(() -> container.getClimberPivot().getClimberDeployDone().get());
        climberLEDs.whileTrue(LedFactory.climbLEDs(container).withName("Climb LEDs"));

        container.getModalControls().algaeClimbMode().and(notifyLowOnTime.negate()).and(climberLEDs.negate())
                .and(() -> getLatestAlgaeStageState().get() == SuperstructureState.STAGE_PROCESSOR)
                .whileTrue(
                        LedFactory.algaeStagingLeds(container,
                                () -> SuperstructureState.STAGE_PROCESSOR)
                                .withName("Algae Climb LEDs Processor"));

        container.getModalControls().algaeClimbMode().and(notifyLowOnTime.negate()).and(climberLEDs.negate())
                .and(() -> getLatestAlgaeStageState().get() == SuperstructureState.REEF_ALGAE_INTAKE_L2)
                .whileTrue(
                        LedFactory.algaeStagingLeds(container,
                                () -> SuperstructureState.REEF_ALGAE_INTAKE_L2)
                                .withName("Algae Climb LEDs L2"));

        container.getModalControls().algaeClimbMode().and(notifyLowOnTime.negate()).and(climberLEDs.negate())
                .and(() -> getLatestAlgaeStageState().get() == SuperstructureState.REEF_ALGAE_INTAKE_L3)
                .whileTrue(
                        LedFactory.algaeStagingLeds(container,
                                () -> SuperstructureState.REEF_ALGAE_INTAKE_L3)
                                .withName("Algae Climb LEDs L3"));

        container.getModalControls().algaeClimbMode().and(notifyLowOnTime.negate()).and(climberLEDs.negate())
                .and(() -> getLatestAlgaeStageState().get() == SuperstructureState.STAGE_BARGE)
                .whileTrue(
                        LedFactory.algaeStagingLeds(container,
                                () -> SuperstructureState.STAGE_BARGE)
                                .withName("Algae Climb LEDs Barge"));

        Trigger turnOffLEDsinDisabled = new Trigger(() -> DriverStation.isDisabled());
        turnOffLEDsinDisabled.onTrue(container.getLeds().commandSolidColor(LedState.kOff)
                .withName("Disabled LEDs Off").ignoringDisable(true));

        Trigger updateLEDsOnEnable = new Trigger(() -> DriverStation.isEnabled());
        updateLEDsOnEnable.onTrue(Commands
                .defer(() -> LedFactory.updateLEDs(container, latestCoralStageState.get(),
                        latestAlgaeStageState.get()), Set.of(container.getLeds()))
                .withName("Update LEDs On Enable"));

        notifyLowOnTime.and(climberLEDs.negate())
                .and(container.getModalControls().groundCoralIntake().negate())
                .and(container.getModalControls().groundCoralIntakeManual().negate())
                .onTrue(LedFactory.lowOnTimeLEDs(container, container.getModalControls().getMode())
                        .withName("Low On Time LEDs"));
    }
}