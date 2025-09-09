package com.team254.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the climber sensor IO. Defines the inputs and methods required for reading climber
 * sensor data.
 */
public interface ClimberSensorIO {
    @AutoLog
    public class ClimberSensorInputs {
        public boolean climberLeftLimitSwitchTriggered;
        public boolean climberRightLimitSwitchTriggered;
    }

    public default void readInputs(ClimberSensorIO.ClimberSensorInputs inputs) {}
}
