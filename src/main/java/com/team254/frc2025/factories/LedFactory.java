package com.team254.frc2025.factories;

import com.team254.frc2025.Constants.LEDConstants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.controlboard.ModalControls.Mode;
import com.team254.frc2025.subsystems.led.LedState;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Factory class for creating commands related to the LED subsystem.
 *
 * <p>This class provides methods to create commands for controlling the robot's LED lighting
 * system. It handles various lighting patterns and animations used for driver feedback, game piece
 * indication, and system status visualization.
 */
public class LedFactory {

    private static final double kLowBattery = LEDConstants.kLowBatteryThresholdVolts;

    public static Command coralModeLEDs(
            RobotContainer container,
            Supplier<SuperstructureState> state,
            BooleanSupplier isGroundIntaking) {
        return new ConditionalCommand(
                groundIntakeLEDs(container),
                coralStagingLEDs(container, state, () -> true),
                isGroundIntaking);
    }

    public static Command updateLEDs(
            RobotContainer container,
            SuperstructureState latestCoralHeight,
            SuperstructureState latestAlgaeHeight) {
        switch (container.getModalControls().getMode()) {
            case CORAL:
                return coralStagingLEDs(container, () -> latestCoralHeight, () -> false);
            case ALGAECLIMB:
                return algaeStagingLeds(container, () -> latestAlgaeHeight);
            case CORALMANUAL:
                return coralStagingLEDs(container, () -> latestCoralHeight, () -> true);
            default:
                return Commands.none();
        }
    }

    public static Command coralModeLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(LedState.kCoralMode);
    }

    public static Command coralManualModeLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(LedState.kCoralManual);
    }

    public static Command groundIntakeLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(LedState.kRed);
    }

    public static Command algaeModeLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(LedState.kGreen);
    }

    public static Command climbLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(LedState.kPurple);
    }

    public static Command latchedLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(LedState.kBlue);
    }

    public static Command batteryLEDs(RobotContainer container, DoubleSupplier voltageSupplier) {
        return container
                .getLeds()
                .commandSolidColor(
                        () -> {
                            if (voltageSupplier.getAsDouble() < kLowBattery) {
                                return LedState.kLowBattery;
                            } else {
                                return LedState.kGoodBattery;
                            }
                        });
    }

    public static Command coralStagingLEDs(
            RobotContainer container,
            Supplier<SuperstructureState> state,
            BooleanSupplier isManual) {
        ConditionalCommand coralManualLeds =
                new ConditionalCommand(
                        container
                                .getLeds()
                                .commandBlinkingState(LedState.kCoralManual, LedState.kOff, 0.25),
                        new ConditionalCommand(
                                container
                                        .getLeds()
                                        .commandSolidPattern(LedState.kL2StagingManualLeds),
                                new ConditionalCommand(
                                        container
                                                .getLeds()
                                                .commandSolidPattern(LedState.kL3StagingManualLeds),
                                        container
                                                .getLeds()
                                                .commandSolidColor(LedState.kCoralManual),
                                        () -> state.get() == SuperstructureState.STAGE_CORAL_L3),
                                () -> state.get() == SuperstructureState.STAGE_CORAL_L2),
                        () -> state.get() == SuperstructureState.STAGE_CORAL_L1);

        ConditionalCommand coralModeLeds =
                new ConditionalCommand(
                        container
                                .getLeds()
                                .commandBlinkingState(LedState.kCoralMode, LedState.kOff, 0.25),
                        new ConditionalCommand(
                                container.getLeds().commandSolidPattern(LedState.kL2StagingLeds),
                                new ConditionalCommand(
                                        container
                                                .getLeds()
                                                .commandSolidPattern(LedState.kL3StagingLeds),
                                        container.getLeds().commandSolidColor(LedState.kCoralMode),
                                        () -> state.get() == SuperstructureState.STAGE_CORAL_L3),
                                () -> state.get() == SuperstructureState.STAGE_CORAL_L2),
                        () -> state.get() == SuperstructureState.STAGE_CORAL_L1);
        return new ConditionalCommand(coralManualLeds, coralModeLeds, isManual);
    }

    public static Command algaeStagingLeds(
            RobotContainer container, Supplier<SuperstructureState> state) {
        return new ConditionalCommand(
                container.getLeds().commandBlinkingState(LedState.kAlgaeMode, LedState.kOff, 0.25),
                new ConditionalCommand(
                        container.getLeds().commandSolidPattern(LedState.kAlgaeL2Leds),
                        new ConditionalCommand(
                                container.getLeds().commandSolidPattern(LedState.kAlgaeL3Leds),
                                container.getLeds().commandSolidColor(LedState.kAlgaeMode),
                                () -> state.get() == SuperstructureState.REEF_ALGAE_INTAKE_L3),
                        () -> state.get() == SuperstructureState.REEF_ALGAE_INTAKE_L2),
                () -> state.get() == SuperstructureState.STAGE_PROCESSOR);
    }

    public static Supplier<LedState> getLedStateByMode(RobotContainer container) {
        return () ->
                container.getModalControls().getMode() == Mode.CORAL
                        ? LedState.kCoralMode
                        : LedState.kCoralManual;
    }

    public static Command lowOnTimeLEDs(RobotContainer container, Mode currentMode) {
        LedState currentLeds = LedState.kOff;
        if (currentMode == Mode.CORAL) {
            currentLeds = LedState.kCoralMode;
        } else if (currentMode == Mode.ALGAECLIMB) {
            currentLeds = LedState.kAlgaeMode;
        } else if (currentMode == Mode.CORALMANUAL) {
            currentLeds = LedState.kCoralManual;
        }
        return container.getLeds().commandBlinkingState(LedState.kPurple, currentLeds, 0.25);
    }
}
