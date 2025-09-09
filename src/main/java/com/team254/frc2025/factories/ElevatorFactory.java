package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating commands related to the elevator subsystem.
 *
 * <p>This class provides methods to create commands for controlling the elevator's vertical
 * movement to specific heights. It supports both blocking and non-blocking movement commands with
 * configurable tolerances, primarily used during scoring and stowing operations.
 */
public class ElevatorFactory {

    public static Command moveToScoringHeightBlocking(
            RobotContainer container, DoubleSupplier elevatorHeightM) {
        return container
                .getElevator()
                .motionMagicSetpointCommandBlocking(
                        elevatorHeightM, Constants.ElevatorConstants.kElevatorToleranceMeters);
    }

    public static Command moveToScoringHeightBlocking(
            RobotContainer container, DoubleSupplier elevatorHeightM, double toleranceM) {
        return container
                .getElevator()
                .motionMagicSetpointCommandBlocking(elevatorHeightM, toleranceM);
    }
}
