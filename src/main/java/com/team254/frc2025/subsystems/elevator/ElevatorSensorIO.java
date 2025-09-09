package com.team254.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code ElevatorSensorIO} interface defines the input/output operations for elevator sensors.
 * It provides methods to read sensor inputs such as hall effect triggers and CAN range finder data.
 */
public interface ElevatorSensorIO {
    @AutoLog
    public class ElevatorSensorInputs {
        public boolean elevatorHallEffectTriggered;
        public double canRangeDistanceLeft;
        public double canRangeSignalStrengthLeft;
        public double canRangeDistanceRight;
        public double canRangeSignalStrengthRight;
    }

    public default void readInputs(ElevatorSensorIO.ElevatorSensorInputs inputs) {}
}
