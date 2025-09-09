package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Factory class for creating commands related to the intake subsystem.
 *
 * <p>This class provides methods to create commands for controlling the intake mechanism, including
 * deploying and retracting the intake, as well as controlling the intake rollers. It handles both
 * standard and lollipop style intake configurations.
 */
public class IntakeFactory {

    public static Command deployIntakeBlocking(RobotContainer container) {
        return container
                .getIntakePivot()
                .motionMagicSetpointCommandBlocking(
                        () -> Constants.IntakeConstants.kIntakePivotDeployPositionRadians,
                        Constants.IntakeConstants.kIntakePivotToleranceRadians);
    }

    public static Command deployLollipopIntakeBlocking(RobotContainer container) {
        return container
                .getIntakePivot()
                .motionMagicSetpointCommandBlocking(
                        () -> Constants.IntakeConstants.kIntakePivotLollipopDeployPositionRadians,
                        Constants.IntakeConstants.kIntakePivotToleranceRadians);
    }

    public static Command stowIntakeBlocking(RobotContainer container) {
        return container
                .getIntakePivot()
                .motionMagicSetpointCommandBlocking(
                        () -> Constants.IntakeConstants.kIntakePivotStowPositionRadians,
                        Constants.IntakeConstants.kIntakePivotToleranceRadians);
    }

    public static Command stowIntakeForClimbBlocking(RobotContainer container) {
        return container
                .getIntakePivot()
                .motionMagicSetpointCommandBlocking(
                        () -> Constants.IntakeConstants.kIntakePivotStowForClimbPositionRadians,
                        Constants.IntakeConstants.kIntakePivotToleranceRadians);
    }

    public static Command spinRollers(RobotContainer container) {
        return container
                .getIntakeRoller()
                .dutyCycleCommand(() -> Constants.IntakeConstants.kIntakeDutyCycleIntake);
    }

    public static Command spinRollersNoEnd(RobotContainer container) {
        return container
                .getIntakeRoller()
                .dutyCycleCommandNoEnd(() -> Constants.IntakeConstants.kIntakeDutyCycleIntake);
    }

    public static Command exhaustRollers(RobotContainer container) {
        return container
                .getIntakeRoller()
                .dutyCycleCommand(() -> Constants.IntakeConstants.kIntakeDutyCycleExhaust);
    }
}
