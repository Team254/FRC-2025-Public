package com.team254.frc2025.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.team254.frc2025.Constants;
import com.team254.lib.util.CTREUtil;

/**
 * Hardware implementation of the climber sensor IO. Handles the actual hardware communication for
 * climber limit switches.
 */
public class ClimberSensorIOHardware implements ClimberSensorIO {

    protected final CANdi climberLimitSwitchCandi;

    protected final StatusSignal<Boolean> climberLeftLimitSwitchTriggered;
    protected final StatusSignal<Boolean> climberRightLimitSwitchTriggered;

    public ClimberSensorIOHardware() {
        climberLimitSwitchCandi =
                new CANdi(
                        Constants.SensorConstants.kClimberLimitSwitchCandiID,
                        Constants.kCanBusDrivebaseClimberCanivore);
        climberLimitSwitchCandi
                .getConfigurator()
                .apply(
                        CTREUtil.createCustomCandiConfiguration(
                                S1CloseStateValue.CloseWhenNotHigh,
                                S1FloatStateValue.PullHigh,
                                S2CloseStateValue.CloseWhenNotHigh,
                                S2FloatStateValue.PullHigh));

        climberLeftLimitSwitchTriggered = climberLimitSwitchCandi.getS1Closed();
        climberRightLimitSwitchTriggered = climberLimitSwitchCandi.getS2Closed();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SensorConstants.kClimberRefreshRateHz,
                climberLeftLimitSwitchTriggered,
                climberRightLimitSwitchTriggered);
    }

    @Override
    public void readInputs(ClimberSensorInputs inputs) {
        BaseStatusSignal.refreshAll(
                climberLeftLimitSwitchTriggered, climberRightLimitSwitchTriggered);
        inputs.climberLeftLimitSwitchTriggered = climberLeftLimitSwitchTriggered.getValue();
        inputs.climberRightLimitSwitchTriggered = climberRightLimitSwitchTriggered.getValue();
    }
}
