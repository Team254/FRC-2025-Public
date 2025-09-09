package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.commands.ChezySequenceCommandGroup;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.ReefscapeUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Factory class for creating commands related to the claw subsystem.
 *
 * <p>This class provides methods to create commands for controlling the claw mechanism, including
 * staging and scoring coral game pieces. It handles the interaction between the claw and coral
 * throughout the intake and scoring processes.
 */
public class ClawFactory {

    public static Command stageCoralInClaw(
            RobotContainer container, AtomicReference<SuperstructureState> latestCoralStageState) {
        return new ChezySequenceCommandGroup(
                        // run until stage banner triggered by front of coral and
                        // second indexer is false
                        new InstantCommand(
                                () ->
                                        container
                                                .getCoralStateTracker()
                                                .updateProcessingInClaw(true)),
                        container
                                .getClaw()
                                .voltageCommand(
                                        () -> Constants.ClawConstants.kCoralRollerVoltageHandoff)
                                .until(() -> container.getClaw().hasCoralAtStageBanner()),
                        // We are now done processing and can move wrist.
                        new InstantCommand(
                                () -> {
                                    container.getCoralStateTracker().updateProcessingInClaw(false);
                                    container
                                            .getCoralStateTracker()
                                            .forceSet(
                                                    CoralStateTracker.CoralPosition.STAGED_IN_CLAW);
                                }),
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new InstantCommand(
                                                () ->
                                                        container
                                                                .getStateMachine()
                                                                .setDesiredState(
                                                                        SuperstructureState
                                                                                .STAGE_CORAL_L1)),
                                        Commands.either(
                                                new InstantCommand(
                                                        () ->
                                                                container
                                                                        .getStateMachine()
                                                                        .setDesiredState(
                                                                                SuperstructureState
                                                                                        .CLEAR_LIMELIGHT)),
                                                Commands.none(),
                                                () ->
                                                        container.getElevator().canStow()
                                                                && ReefscapeUtil.awayFromReef(
                                                                        container)),
                                        () ->
                                                latestCoralStageState.get()
                                                        == SuperstructureState.STAGE_CORAL_L1),
                                new InstantCommand(
                                        () ->
                                                container
                                                        .getStateMachine()
                                                        .applyFutureDesiredState()),
                                () ->
                                        container.getStateMachine().getFutureDesiredState() == null
                                                && container.getStateMachine().getDesiredState()
                                                        == container
                                                                .getStateMachine()
                                                                .getCurrentState()))
                .finallyDo(
                        () -> {
                            container.getCoralStateTracker().updateProcessingInClaw(false);
                        })
                .withName("Stage Coral In Claw");
    }

    public static Command scoreCoral(
            RobotContainer container, Supplier<SuperstructureState> superstructureState) {
        return Commands.defer(
                () -> {
                    return container
                            .getClaw()
                            .dutyCycleCommand(
                                    () -> Constants.ClawConstants.kCoralRollerDutyCycleScore)
                            .withName("Score Coral");
                },
                Set.of(container.getClaw()));
    }

    public static Command scoreL1Deep(RobotContainer container) {
        var setpoint =
                Constants.ClawConstants.kCoralRollerL1PrescoreRotations
                        + container.getClaw().getPositionRotations();
        return container
                .getClaw()
                .positionSetpointUntilOnTargetCommand(
                        () -> setpoint,
                        () ->
                                Math.abs(Constants.ClawConstants.kCoralRollerL1PrescoreRotations)
                                        * .1)
                .andThen(
                        container
                                .getClaw()
                                .velocitySetpointCommand(
                                        () -> Constants.ClawConstants.kCoralRollerL1DeepRPS, 1))
                .alongWith(
                        Commands.runOnce(
                                () ->
                                        container
                                                .getCoralStateTracker()
                                                .forceSet(CoralPosition.NONE)));
    }

    public static Command scoreL1Shallow(RobotContainer container) {
        var setpoint =
                Constants.ClawConstants.kCoralRollerL1PrescoreRotations
                        + container.getClaw().getPositionRotations();
        return container
                .getClaw()
                .positionSetpointUntilOnTargetCommand(
                        () -> setpoint,
                        () ->
                                Math.abs(Constants.ClawConstants.kCoralRollerL1PrescoreRotations)
                                        * .1)
                .andThen(
                        container
                                .getClaw()
                                .velocitySetpointCommand(
                                        () -> Constants.ClawConstants.kCoralRollerL1ShallowRPS, 1))
                .alongWith(
                        Commands.runOnce(
                                () ->
                                        container
                                                .getCoralStateTracker()
                                                .forceSet(CoralPosition.NONE)));
    }

    public static Command exhaustCoral(RobotContainer container) {
        return container
                .getClaw()
                .dutyCycleCommand(() -> Constants.ClawConstants.kCoralRollerDutyCycleExhaust)
                .withName("Exhaust Coral");
    }

    public static Command intakeAlgae(RobotContainer container) {
        return container
                .getClaw()
                .setTorqueCurrentFOC(() -> Constants.ClawConstants.kAlgaeRollerTorqueCurrentIntake)
                .withName("intakeAlgae");
    }

    public static Command holdAlgae(RobotContainer container) {
        return container
                .getClaw()
                .setTorqueCurrentFOC(() -> Constants.ClawConstants.kAlgaeRollerTorqueCurrentHold);
    }

    public static Command scoreAlgaeBarge(RobotContainer container) {
        return container
                .getClaw()
                .voltageCommand(() -> Constants.ClawConstants.kAlgaeRollerVoltageScoreBarge)
                .withTimeout(0.4)
                .andThen(
                        new InstantCommand(
                                () ->
                                        container
                                                .getClaw()
                                                .updateLastAlgaeScoreTimestamp(
                                                        RobotTime.getTimestampSeconds())))
                .withName("Score Algae Barge");
    }

    public static Command scoreAlgaeProcessor(RobotContainer container) {
        return container
                .getClaw()
                .dutyCycleCommand(() -> -1)
                .finallyDo(
                        () ->
                                container
                                        .getClaw()
                                        .updateLastAlgaeScoreTimestamp(
                                                RobotTime.getTimestampSeconds()))
                .withName("Score Algae Processor");
    }

    public static Command exhaustAlgae(RobotContainer container) {
        return container
                .getClaw()
                .setTorqueCurrentFOC(() -> Constants.ClawConstants.kAlgaeRollerTorqueCurrentExhaust)
                .withTimeout(0.5)
                .andThen(
                        new InstantCommand(
                                () ->
                                        container
                                                .getClaw()
                                                .updateLastAlgaeScoreTimestamp(
                                                        RobotTime.getTimestampSeconds())))
                .withName("exhaust algae");
    }

    public static Command algaeIntakingEnd(RobotContainer container) {
        return container
                .getClaw()
                .setTorqueCurrentFOC(() -> Constants.ClawConstants.kAlgaeRollerTorqueCurrentIntake)
                .withTimeout(Constants.ClawConstants.kAlgaeRollerDebounceTime + 0.1)
                .onlyIf(() -> !container.getClaw().hasAlgae())
                .withName("algaeIntakingEnd");
    }
}
