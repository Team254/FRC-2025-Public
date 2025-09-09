package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Factory class for creating commands related to the indexer subsystem.
 *
 * <p>This class provides methods to create commands for controlling the indexer mechanism,
 * including running it forward for normal operation and in reverse for ejecting game pieces. It
 * handles the movement of game pieces through the robot's scoring system.
 */
public class IndexerFactory {

    public static Command runIndexer(RobotContainer container) {
        return container
                .getIndexer()
                .dutyCycleCommand(() -> Constants.IndexerConstants.kIndexerDutyCycle);
    }

    public static Command exhaustIndexer(RobotContainer container) {
        return container
                .getIndexer()
                .dutyCycleCommand(() -> Constants.IndexerConstants.kIndexerDutyCycleExhaust);
    }
}
