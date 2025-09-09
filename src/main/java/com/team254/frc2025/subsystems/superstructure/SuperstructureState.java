package com.team254.frc2025.subsystems.superstructure;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.factories.SuperstructureFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.Function;

public enum SuperstructureState {
    CLEAR_ELEVATOR_CROSSBEAM(
            Constants.ElevatorConstants.kClearElevatorCrossbeamHeightMeters,
            Constants.WristConstants.kClearElevatorCrossbeamMinPositionRadians,
            container -> {
                double desiredWristAngle =
                        container.getStateMachine().getDesiredState().getWristAngleRad();
                double commandedWristAngle;
                Command moveCommand;
                if (desiredWristAngle
                        < Constants.WristConstants.kClearElevatorCrossbeamMinPositionRadians) {
                    commandedWristAngle =
                            Constants.WristConstants.kClearElevatorCrossbeamMinPositionRadians;
                    moveCommand =
                            SuperstructureFactory.moveToSuperstructureLevelTolerance(
                                    container,
                                    () ->
                                            Constants.ElevatorConstants
                                                    .kClearElevatorCrossbeamHeightMeters,
                                    () -> commandedWristAngle,
                                    0.4);
                } else if (desiredWristAngle
                        > Constants.WristConstants.kClearElevatorCrossbeamMaxPositionRadians) {
                    commandedWristAngle =
                            Constants.WristConstants.kClearElevatorCrossbeamMaxPositionRadians;
                    moveCommand =
                            SuperstructureFactory.moveToSuperstructureLevelToleranceNoWristCancel(
                                    container,
                                    () ->
                                            Constants.ElevatorConstants
                                                    .kClearElevatorCrossbeamHeightMeters,
                                    () -> commandedWristAngle);
                } else {
                    commandedWristAngle = desiredWristAngle;
                    moveCommand =
                            SuperstructureFactory.moveToSuperstructureLevelTolerance(
                                    container,
                                    () ->
                                            Constants.ElevatorConstants
                                                    .kClearElevatorCrossbeamHeightMeters,
                                    () -> commandedWristAngle,
                                    0.4);
                }

                return moveCommand;
            }),
    CLEAR_WHEELS(
            Constants.ElevatorConstants.kClearWheelsHeightMeters,
            Constants.WristConstants.kClearWheelsPositionRadians),
    CLEAR_LIMELIGHT(
            Constants.ElevatorConstants.kClearLimelightHeightMeters,
            Constants.WristConstants.kClearLimelightPositionRadians),
    STOW_CORAL(
            Constants.ElevatorConstants.kElevatorStowCoralHeightMeters,
            Constants.WristConstants.kWristStowCoralPositionRadians),
    STOW_ALGAE(
            Constants.ElevatorConstants.kElevatorStowAlgaeHeightMeters,
            Constants.WristConstants.kWristStowAlgaePositionRadians),
    GROUND_ALGAE_INTAKE(
            Constants.ElevatorConstants.kElevatorGroundAlgaeIntakeHeightMeters,
            Constants.WristConstants.kWristGroundAlgaeIntakePositionRadians),
    LOLLIPOP_INTAKE(
            Constants.ElevatorConstants.kElevatorLollipopIntakeHeightMeters,
            Constants.WristConstants.kWristLollipopIntakePositionRadians),
    REEF_ALGAE_SLAP(
            Constants.ElevatorConstants.kElevatorReefSlap,
            Constants.WristConstants.kWristReefAlgaeIntakeL2PositionRadians),
    REEF_ALGAE_INTAKE_L2(
            Constants.ElevatorConstants.kElevatorReefAlgaeIntakeL2HeightMeters,
            Constants.WristConstants.kWristReefAlgaeIntakeL2PositionRadians),
    REEF_ALGAE_INTAKE_L3(
            Constants.ElevatorConstants.kElevatorReefAlgaeIntakeL3HeightMeters,
            Constants.WristConstants.kWristReefAlgaeIntakeL3PositionRadians),
    REEF_ALGAE_INTAKE_L3_MANUAL(
            Constants.ElevatorConstants.kElevatorReefAlgaeIntakeL3ManualHeightMeters,
            Constants.WristConstants.kWristReefAlgaeIntakeL3ManualPositionRadians),
    STAGE_CORAL_L1(
            Constants.ElevatorConstants.kElevatorL1HeightMeters,
            Constants.WristConstants.kWristL1PositionRadians),
    STAGE_CORAL_L2(
            Constants.ElevatorConstants.kElevatorL2HeightMeters,
            Constants.WristConstants.kWristL2PositionRadians),
    NON_DESCORE_CORAL_L2(
            Constants.ElevatorConstants.kElevatorL2NonDescoreHeightMeters,
            Constants.WristConstants.kWristL2NonDescorePositionRadians),
    STAGE_CORAL_L3(
            Constants.ElevatorConstants.kElevatorL3HeightMeters,
            Constants.WristConstants.kWristL3PositionRadians),
    NON_DESCORE_CORAL_L3(
            Constants.ElevatorConstants.kElevatorL3NonDescoreHeightMeters,
            Constants.WristConstants.kWristL3NonDescorePositionRadians),
    STAGE_CORAL_L4(
            Constants.ElevatorConstants.kElevatorL4HeightMeters,
            Constants.WristConstants.kWristL4PositionRadians),
    INTERMEDIATE_CORAL_L4(
            Constants.ElevatorConstants.kElevatorL4IntermediateHeightMeters,
            Constants.WristConstants.kWristL4IntermediatePositionRadians),
    NON_DESCORE_CORAL_L4(
            Constants.ElevatorConstants.kElevatorL4NonDescoreHeightMeters,
            Constants.WristConstants.kWristL4NonDescorePositionRadians),
    INTERMEDIATE_BARGE(
            Constants.ElevatorConstants.kElevatorBargeHeightMeters,
            Constants.WristConstants.kWristStowAlgaePositionRadians),
    STAGE_PROCESSOR(
            Constants.ElevatorConstants.kElevatorProcessorHeightMeters,
            Constants.WristConstants.kWristProcessorPositionRadians),
    CLEAR_BUMPER(
            Constants.ElevatorConstants.kElevatorStowAlgaeHeightMeters,
            Constants.WristConstants.kWristGroundAlgaeIntakePositionRadians),
    STOW_ELEVATOR(
            Constants.ElevatorConstants.kElevatorStowCoralHeightMeters,
            Constants.WristConstants.kWristL3NonDescorePositionRadians),
    STAGE_BARGE(
            Constants.ElevatorConstants.kElevatorBargeHeightMeters,
            Constants.WristConstants.kWristScoreBargePositionRadians);

    private final Function<RobotContainer, Command> commandSupplier;
    private final double elevatorHeightMeters;
    private final double wristAngleRad;

    SuperstructureState(
            double elevatorHeightMeters,
            double wristAngleRad,
            Function<RobotContainer, Command> commandSupplier) {
        this.elevatorHeightMeters = elevatorHeightMeters;
        this.wristAngleRad = wristAngleRad;
        this.commandSupplier = commandSupplier;
    }

    SuperstructureState(double elevatorHeightMeters, double wristAngleRad) {
        this.elevatorHeightMeters = elevatorHeightMeters;
        this.wristAngleRad = wristAngleRad;
        this.commandSupplier =
                container ->
                        SuperstructureFactory.moveToSuperstructureLevel(
                                container, () -> elevatorHeightMeters, () -> wristAngleRad);
    }

    public double getElevatorHeightMeters() {
        return elevatorHeightMeters;
    }

    public double getWristAngleRad() {
        return wristAngleRad;
    }

    public Command getCommand(RobotContainer container) {
        if (commandSupplier == null) {
            return Commands.none();
        }
        return commandSupplier.apply(container);
    }

    public Set<SuperstructureState> allowedNextStates() {
        Set<SuperstructureState> all = EnumSet.allOf(SuperstructureState.class);
        switch (this) {
            case STOW_CORAL:
                all.removeAll(EnumSet.of(INTERMEDIATE_BARGE, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case STOW_ELEVATOR:
                all.removeAll(
                        EnumSet.of(
                                INTERMEDIATE_BARGE, NON_DESCORE_CORAL_L4, STAGE_BARGE, STOW_CORAL));
                return all;
            case CLEAR_WHEELS:
                all.removeAll(EnumSet.of(INTERMEDIATE_BARGE, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case CLEAR_LIMELIGHT:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case GROUND_ALGAE_INTAKE:
                all.removeAll(EnumSet.of(STAGE_BARGE, NON_DESCORE_CORAL_L4));
            case LOLLIPOP_INTAKE:
                all.removeAll(EnumSet.of(STOW_CORAL, STAGE_BARGE, NON_DESCORE_CORAL_L4));
                return all;
            case CLEAR_BUMPER:
                return EnumSet.of(STOW_ALGAE, GROUND_ALGAE_INTAKE);
            case STOW_ALGAE:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case REEF_ALGAE_SLAP:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case REEF_ALGAE_INTAKE_L2:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case REEF_ALGAE_INTAKE_L3:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case REEF_ALGAE_INTAKE_L3_MANUAL:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case STAGE_CORAL_L1:
                all.removeAll(
                        EnumSet.of(
                                INTERMEDIATE_BARGE, STAGE_BARGE, STOW_CORAL, NON_DESCORE_CORAL_L4));
                return all;
            case STAGE_CORAL_L2:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case NON_DESCORE_CORAL_L2:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case STAGE_CORAL_L3:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case NON_DESCORE_CORAL_L3:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case STAGE_CORAL_L4:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case INTERMEDIATE_CORAL_L4:
                return EnumSet.of(NON_DESCORE_CORAL_L4);
            case NON_DESCORE_CORAL_L4:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case INTERMEDIATE_BARGE:
                return EnumSet.of(CLEAR_ELEVATOR_CROSSBEAM, STAGE_BARGE);
            case STAGE_PROCESSOR:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case CLEAR_ELEVATOR_CROSSBEAM:
                all.removeAll(EnumSet.of(STOW_CORAL, NON_DESCORE_CORAL_L4, STAGE_BARGE));
                return all;
            case STAGE_BARGE:
                return EnumSet.of(CLEAR_ELEVATOR_CROSSBEAM);
            default:
                return all;
        }
    }

    public SuperstructureState getAvoidDescoreState() {
        switch (this) {
            case STAGE_CORAL_L2:
                return NON_DESCORE_CORAL_L2;
            case STAGE_CORAL_L3:
                return NON_DESCORE_CORAL_L3;
            case STAGE_CORAL_L4:
                return NON_DESCORE_CORAL_L4;
            default:
                return this;
        }
    }

    public double getContinueAutoAlignElevatorPosition() {
        switch (this) {
            case STAGE_CORAL_L2:
                return Constants.ElevatorConstants.kElevatorL2AutoAlignContinueHeightMeters;
            case STAGE_CORAL_L3:
                return Constants.ElevatorConstants.kElevatorL3AutoAlignContinueHeightMeters;
            case STAGE_CORAL_L4:
                return Constants.ElevatorConstants.kElevatorL4AutoAlignContinueHeightMeters;
            default:
                return 0.0;
        }
    }

    public boolean isCoralState() {
        switch (this) {
            case STOW_CORAL:
            case STAGE_CORAL_L1:
            case STAGE_CORAL_L2:
            case STAGE_CORAL_L3:
            case STAGE_CORAL_L4:
                return true;
            default:
                return false;
        }
    }

    public boolean isAlgaeState() {
        switch (this) {
            case GROUND_ALGAE_INTAKE:
            case LOLLIPOP_INTAKE:
            case STOW_ALGAE:
            case REEF_ALGAE_SLAP:
            case REEF_ALGAE_INTAKE_L2:
            case REEF_ALGAE_INTAKE_L3:
            case REEF_ALGAE_INTAKE_L3_MANUAL:
            case STAGE_PROCESSOR:
            case INTERMEDIATE_BARGE:
                return true;
            default:
                return false;
        }
    }
}
