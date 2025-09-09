package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.function.DoubleSupplier;

/**
 * The {@code SuperstructureFactory} class is responsible for creating and managing various commands
 * related to the robot's superstructure. It provides factory methods for creating complex sequences
 * of commands for actions like intaking, scoring, and exhausting game pieces.
 *
 * <p>This class handles the coordination of multiple subsystems such as the intake, indexer, claw,
 * elevator, and wrist. It ensures that these subsystems work together seamlessly to perform the
 * desired actions during a match.
 *
 * <p>Key functionalities include:
 *
 * <ul>
 *   <li>Intaking game pieces and preparing them for scoring
 *   <li>Scoring game pieces with precise control over subsystems
 *   <li>Handling special scenarios such as exhausting game pieces
 *   <li>Managing subsystem states to achieve specific robot behaviors
 * </ul>
 *
 * <p>The methods in this class return {@link Command} objects that can be scheduled or combined
 * with other commands to create comprehensive autonomous or teleoperated sequences.
 */
public class SuperstructureFactory {

    public static Command delayedScore(RobotContainer container) {
        return Commands.sequence(
                new ConditionalCommand(
                        Commands.waitUntil(() -> container.getClaw().hasAlgae()),
                        Commands.waitUntil(
                                () ->
                                        container.getCoralStateTracker().getCurrentPosition()
                                                == CoralPosition.STAGED_IN_CLAW),
                        () -> {
                            SuperstructureState state =
                                    container.getStateMachine().getCurrentState();
                            return state == SuperstructureState.STAGE_PROCESSOR
                                    || state == SuperstructureState.STAGE_BARGE;
                        }));
    }

    /** Scores game piece based on current state. */
    public static Command stationaryScore(RobotContainer container) {
        return new ConditionalCommand(
                        ClawFactory.scoreAlgaeBarge(container).withTimeout(0.5),
                        ClawFactory.scoreCoral(
                                        container,
                                        () -> container.getStateMachine().getCurrentState())
                                .withTimeout(1.5),
                        () -> {
                            SuperstructureState state =
                                    container.getStateMachine().getCurrentState();
                            return state == SuperstructureState.STAGE_PROCESSOR
                                    || state == SuperstructureState.STAGE_BARGE;
                        })
                .withName("StationaryScore");
    }

    /** Intakes algae from ground if claw is empty. Stops when algae is detected. */
    public static Command groundIntakeAlgae(RobotContainer container) {
        return new ConditionalCommand(
                        ClawFactory.intakeAlgae(container)
                                .until(() -> container.getClaw().hasAlgae()),
                        Commands.none(),
                        () -> !container.getClaw().hasAlgae())
                .withName("Superstructure Intake Algae Ground");
    }

    /** Expels coral pieces using the claw's scoring mechanism. */
    public static Command exhaustClawSide(RobotContainer container) {
        return ClawFactory.scoreCoral(
                        container, () -> container.getStateMachine().getCurrentState())
                .withName("Exhaust Coral Claw Side");
    }

    /** Stows intake and runs rollers/indexer in reverse to expel coral. */
    public static Command exhaustIntakeSide(RobotContainer container) {
        return IntakeFactory.stowIntakeBlocking(container)
                .andThen(
                        new ParallelCommandGroup(
                                IntakeFactory.exhaustRollers(container),
                                IndexerFactory.exhaustIndexer(container)))
                .withName("Exhaust Coral Intake Side");
    }

    /** Intakes coral from ground using intake and indexer when system is empty. */
    public static Command groundIntake(RobotContainer container) {
        return new ConditionalCommand(
                Commands.parallel(
                                IntakeFactory.deployIntakeBlocking(container),
                                IntakeFactory.spinRollers(container),
                                IndexerFactory.runIndexer(container))
                        .until(() -> container.getIndexer().hasCoralAtFirstIndexerBanner())
                        .withName("Superstructure Ground Intake"),
                Commands.none(),
                () -> container.getCoralStateTracker().getCurrentPosition() == CoralPosition.NONE);
    }

    /** Intakes coral with continuous roller operation after detection. */
    public static Command groundIntakeNoEnd(RobotContainer container) {
        return new ConditionalCommand(
                Commands.parallel(
                                IntakeFactory.deployIntakeBlocking(container),
                                IntakeFactory.spinRollersNoEnd(container),
                                IndexerFactory.runIndexer(container))
                        .until(() -> container.getIndexer().hasCoralAtFirstIndexerBanner())
                        .withName("Superstructure Ground Intake"),
                Commands.none(),
                () -> container.getCoralStateTracker().getCurrentPosition() == CoralPosition.NONE);
    }

    /** Intakes coral using only intake rollers, no indexer. */
    public static Command groundIntakeNoIndexerNoEnd(RobotContainer container) {
        return new ConditionalCommand(
                        Commands.parallel(
                                        IntakeFactory.deployIntakeBlocking(container),
                                        IntakeFactory.spinRollersNoEnd(container))
                                .until(() -> container.getIndexer().hasCoralAtFirstIndexerBanner())
                                .withName("Superstructure Ground Intake"),
                        Commands.none(),
                        () ->
                                container.getCoralStateTracker().getCurrentPosition()
                                        == CoralPosition.NONE)
                .withName("Ground Intake no Indexer");
    }

    /** Intakes coral from lollipop position. */
    public static Command lollipopIntake(RobotContainer container) {
        return new ConditionalCommand(
                Commands.parallel(
                                IntakeFactory.deployLollipopIntakeBlocking(container),
                                IntakeFactory.spinRollers(container))
                        .until(() -> container.getIndexer().hasCoralAtFirstIndexerBanner())
                        .withName("Superstructure Ground Intake"),
                Commands.none(),
                () -> container.getCoralStateTracker().getCurrentPosition() == CoralPosition.NONE);
    }

    /** Moves elevator and wrist to target positions in parallel. */
    public static Command moveToSuperstructureLevel(
            RobotContainer container,
            DoubleSupplier elevatorHeightM,
            DoubleSupplier wristAngleRad) {
        return Commands.parallel(
                ElevatorFactory.moveToScoringHeightBlocking(container, elevatorHeightM),
                WristFactory.moveToScoringAngleBlocking(container, wristAngleRad));
    }

    /** Moves to target positions with configurable elevator tolerance. */
    public static Command moveToSuperstructureLevelTolerance(
            RobotContainer container,
            DoubleSupplier elevatorHeightM,
            DoubleSupplier wristAngleRad,
            double elevatorToleranceM) {
        return Commands.deadline(
                ElevatorFactory.moveToScoringHeightBlocking(
                                container, elevatorHeightM, elevatorToleranceM)
                        .alongWith(
                                Commands.waitUntil(
                                        () ->
                                                container.getWrist().getCurrentPosition()
                                                        >= Constants.WristConstants
                                                                .kClearElevatorCrossbeamMinPositionRadians)),
                WristFactory.moveToScoringAngleBlocking(container, wristAngleRad));
    }

    /** Moves to target positions with wrist movement priority. */
    public static Command moveToSuperstructureLevelToleranceNoWristCancel(
            RobotContainer container,
            DoubleSupplier elevatorHeightM,
            DoubleSupplier wristAngleRad) {
        return Commands.deadline(
                WristFactory.moveToScoringAngleBlocking(container, wristAngleRad),
                ElevatorFactory.moveToScoringHeightBlocking(container, elevatorHeightM));
    }
}
