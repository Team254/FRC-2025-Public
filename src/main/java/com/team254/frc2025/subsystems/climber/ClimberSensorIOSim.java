package com.team254.frc2025.subsystems.climber;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;

/**
 * Simulation implementation of the climber sensor IO. Simulates the behavior of the climber limit
 * switches for testing purposes.
 */
public class ClimberSensorIOSim extends ClimberSensorIOHardware {

    CANdiSimState climberHallEffectCandiSim;
    CANdiSimState climberLimitSwitchCandiSim;

    public ClimberSensorIOSim() {
        super();
        climberLimitSwitchCandiSim = new CANdiSimState(climberLimitSwitchCandi);

        // Initialize with no climber limit switches triggered
        setLeftClimberLimitSwitchUntriggered();
        setRightClimberLimitSwitchUntriggered();
    }

    public void setLeftClimberLimitSwitchUntriggered() {
        climberLimitSwitchCandiSim.setS1State(S1StateValue.High);
    }

    public void setLeftClimberLimitSwitchTriggered() {
        climberLimitSwitchCandiSim.setS1State(S1StateValue.Low);
    }

    public void setRightClimberLimitSwitchUntriggered() {
        climberLimitSwitchCandiSim.setS2State(S2StateValue.High);
    }

    public void setRightClimberLimitSwitchTriggered() {
        climberLimitSwitchCandiSim.setS2State(S2StateValue.Low);
    }
}
