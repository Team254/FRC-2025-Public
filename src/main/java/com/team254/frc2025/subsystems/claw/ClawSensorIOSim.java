package com.team254.frc2025.subsystems.claw;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;

/**
 * The {@code ClawSensorIOSim} class extends the {@link ClawSensorIOHardware} and provides
 * simulation-specific functionality for the claw sensor system. This class allows for simulating
 * the state of the claw's banner sensors.
 */
public class ClawSensorIOSim extends ClawSensorIOHardware {

    CANdiSimState clawCoralCandiSim;

    public ClawSensorIOSim() {
        super();
        clawCoralCandiSim = new CANdiSimState(clawCoralCandi);

        // we start with no piece
        setScoreCoralBannerUntriggered();
        setStageCoralBannerUntriggered();
    }

    public void setStageCoralBannerUntriggered() {
        clawCoralCandiSim.setS2State(S2StateValue.High);
    }

    public void setStageCoralBannerTriggered() {
        clawCoralCandiSim.setS2State(S2StateValue.Low);
    }

    public void setScoreCoralBannerUntriggered() {
        clawCoralCandiSim.setS1State(S1StateValue.High);
    }

    public void setScoreCoralBannerTriggered() {
        clawCoralCandiSim.setS1State(S1StateValue.Low);
    }
}
