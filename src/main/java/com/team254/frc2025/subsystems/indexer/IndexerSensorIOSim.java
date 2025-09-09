package com.team254.frc2025.subsystems.indexer;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;

/**
 * The {@code IndexerSensorIOSim} class extends {@link IndexerSensorIOHardware} to provide
 * simulation-specific functionality for indexer banner sensors. It allows for testing indexer
 * behavior without requiring physical hardware.
 */
public class IndexerSensorIOSim extends IndexerSensorIOHardware {

    CANdiSimState indexerCandiSim;

    public IndexerSensorIOSim() {
        super();
        indexerCandiSim = new CANdiSimState(indexerCandi);

        setFirstIndexerBannerUntriggered();
        setSecondIndexerBannerUntriggered();
    }

    public void setFirstIndexerBannerUntriggered() {
        indexerCandiSim.setS1State(S1StateValue.High);
    }

    public void setFirstIndexerBannerTriggered() {
        indexerCandiSim.setS1State(S1StateValue.Low);
    }

    public void setSecondIndexerBannerUntriggered() {
        indexerCandiSim.setS2State(S2StateValue.High);
    }

    public void setSecondIndexerBannerTriggered() {
        indexerCandiSim.setS2State(S2StateValue.Low);
    }
}
