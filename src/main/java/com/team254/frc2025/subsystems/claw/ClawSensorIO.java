package com.team254.frc2025.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code ClawSensorIO} interface defines the input/output operations for the claw sensor
 * system. It is responsible for reading sensor inputs from the claw's banner sensors.
 */
public interface ClawSensorIO {
    @AutoLog
    public class ClawSensorInputs {
        public boolean stageCoralBannerTriggered;
        public boolean scoreCoralBannerTriggered;
    }

    public default void readInputs(ClawSensorIO.ClawSensorInputs inputs) {}
}
