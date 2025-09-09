package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for creating commands related to the climber subsystem.
 *
 * <p>This class provides methods to create commands for controlling the climber mechanism,
 * including deploying, latching, and retracting climber components during the endgame phase. It
 * handles the automation of the climbing sequence for consistent and reliable performance.
 */
public class ClimberFactory {

    public static Command autoClimb(RobotContainer container) {
        return Commands.sequence(
                // fully latched when both limit switches are triggered
                Commands.parallel(deployClimber(container), latchClimber(container))
                        .until(
                                () ->
                                        container
                                                        .getClimberRoller()
                                                        .climberLeftLimitSwitchTriggered()
                                                && container
                                                        .getClimberRoller()
                                                        .climberRightLimitSwitchTriggered())
                        .alongWith(
                                new InstantCommand(
                                        () ->
                                                Logger.recordOutput(
                                                        "Auto Climb Sequence", "deploying"))),
                stowClimber(container)
                        .withName("Auto Climb")
                        .alongWith(
                                new InstantCommand(
                                        () ->
                                                Logger.recordOutput(
                                                        "Auto Climb Sequence", "climbing"))));
    }

    public static Command manualClimb(RobotContainer container) {
        return new ParallelCommandGroup(deployClimber(container), latchClimber(container))
                .withName("Manual Climb");
    }

    public static Command deployClimber(RobotContainer container) {
        // run until deployed
        return new SequentialCommandGroup(
                        container
                                .getClimberPivot()
                                .voltageCommand(
                                        () -> Constants.ClimberConstants.kClimberPivotVoltageDeploy)
                                .until(
                                        () ->
                                                container.getClimberPivot().getRawRotorPosition()
                                                        >= Constants.ClimberConstants
                                                                        .kClimberPivotDeployRotations
                                                                + container
                                                                        .getClimberPivot()
                                                                        .getRawPositionOffset()),
                        // run until slack removed
                        new InstantCommand(
                                () -> container.getClimberPivot().setClimberDeployDone(true)))
                .onlyIf(() -> !container.getClimberPivot().getClimberDeployDone().get())
                .withName("Deploy Climber");
    }

    public static Command stowClimber(RobotContainer container) {
        return container
                .getClimberPivot()
                .voltageCommand(() -> Constants.ClimberConstants.kClimberPivotVoltageStow)
                .until(
                        () ->
                                container.getClimberPivot().getCurrentPosition()
                                        <= Units.degreesToRadians(2))
                .withName("Stow Climber");
    }

    public static Command manualJogClimber(RobotContainer container) {
        return container
                .getClimberPivot()
                .voltageCommand(() -> Constants.ClimberConstants.kClimberPivotVoltageStow)
                .withName("Manual Jog Climber");
    }

    public static Command latchClimber(RobotContainer container) {
        return container
                .getClimberRoller()
                .dutyCycleCommand(() -> Constants.ClimberConstants.kClimberRollerDutyCycleLatch);
    }
}
