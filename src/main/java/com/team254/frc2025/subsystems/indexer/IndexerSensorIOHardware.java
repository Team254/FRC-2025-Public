package com.team254.frc2025.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.team254.frc2025.Constants;
import com.team254.lib.util.CTREUtil;

/**
 * The {@code IndexerSensorIOHardware} class implements the {@link IndexerSensorIO} interface to
 * provide hardware-level access to indexer banner sensors. It handles the configuration and reading
 * of these sensors for detecting game pieces.
 */
public class IndexerSensorIOHardware implements IndexerSensorIO {

    protected final CANdi indexerCandi;

    protected final StatusSignal<Boolean> firstIndexerBannerTriggered;
    protected final StatusSignal<Boolean> secondIndexerBannerTriggered;

    public IndexerSensorIOHardware() {
        indexerCandi =
                new CANdi(
                        Constants.SensorConstants.kIndexerCandiID,
                        Constants.kCanBusSuperstructureCanivore);
        indexerCandi.getConfigurator().apply(CTREUtil.createCandiConfiguration());

        firstIndexerBannerTriggered = indexerCandi.getS1Closed();
        secondIndexerBannerTriggered = indexerCandi.getS2Closed();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SensorConstants.kIndexerRefreshRateHz,
                firstIndexerBannerTriggered,
                secondIndexerBannerTriggered);
    }

    @Override
    public void readInputs(IndexerSensorInputs inputs) {
        BaseStatusSignal.refreshAll(firstIndexerBannerTriggered, secondIndexerBannerTriggered);

        inputs.firstIndexerBannerTriggered = firstIndexerBannerTriggered.getValue();
        inputs.secondIndexerBannerTriggered = secondIndexerBannerTriggered.getValue();
    }
}
