package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class WristFactory {

    public static Command moveToScoringAngleBlocking(
            RobotContainer container, DoubleSupplier wristAngleRad) {
        return Commands.defer(
                () ->
                        container
                                .getWrist()
                                .motionMagicSetpointCommandBlocking(
                                        wristAngleRad,
                                        () -> {
                                            if (container.getClaw().hasAlgae()) {
                                                return Constants.kHasAlgaeWristConfig;
                                            } else {
                                                return Constants.kDefaultWristConfig;
                                            }
                                        },
                                        () -> {
                                            if (container.getWrist().getCurrentPosition()
                                                            < Math.toRadians(-80)
                                                    && container.getWrist().getCurrentPosition()
                                                            > Math.toRadians(-100)) {
                                                if (wristAngleRad.getAsDouble()
                                                        > container
                                                                .getWrist()
                                                                .getCurrentPosition()) {
                                                    // If we are raising, do nothing
                                                    return 0.0;
                                                } else {
                                                    return -1.0;
                                                }
                                            } else {
                                                return 0.0;
                                            }
                                        },
                                        Constants.WristConstants.kWristToleranceRadians,
                                        wristAngleRad.getAsDouble() < Math.toRadians(-80) ? 1 : 0),
                Set.of(container.getWrist()));
    }
}
