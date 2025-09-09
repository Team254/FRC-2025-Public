package com.team254.frc2025.subsystems.claw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.team254.frc2025.Constants;
import com.team254.lib.util.CTREUtil;

/**
 * The {@code ClawSensorIOHardware} class implements the {@link ClawSensorIO} interface and provides
 * hardware-level control for the claw sensor system. It manages the CANdi device and its associated
 * banner sensors for detecting game pieces.
 */
public class ClawSensorIOHardware implements ClawSensorIO {

    protected final CANdi clawCoralCandi;

    protected final StatusSignal<Boolean> stageCoralBannerTriggered;
    protected final StatusSignal<Boolean> scoreCoralBannerTriggered;

    public ClawSensorIOHardware() {
        clawCoralCandi =
                new CANdi(
                        Constants.SensorConstants.kClawCoralCandiID,
                        Constants.kCanBusSuperstructureCanivore);
        clawCoralCandi.getConfigurator().apply(CTREUtil.createCandiConfiguration());

        stageCoralBannerTriggered = clawCoralCandi.getS2Closed();
        scoreCoralBannerTriggered = clawCoralCandi.getS1Closed();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SensorConstants.kClawRefreshRateHz,
                stageCoralBannerTriggered,
                scoreCoralBannerTriggered);
    }

    @Override
    public void readInputs(ClawSensorInputs inputs) {
        BaseStatusSignal.refreshAll(stageCoralBannerTriggered, scoreCoralBannerTriggered);

        inputs.stageCoralBannerTriggered = stageCoralBannerTriggered.getValue();
        inputs.scoreCoralBannerTriggered = scoreCoralBannerTriggered.getValue();
    }
}
