package com.team254.frc2025.subsystems.elevator;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.CANrangeSimState;

/**
 * The {@code ElevatorSensorIOSim} class extends {@link ElevatorSensorIOHardware} to provide
 * simulation-specific functionality for elevator sensors. It allows for testing elevator behavior
 * without requiring physical hardware.
 */
public class ElevatorSensorIOSim extends ElevatorSensorIOHardware {

    private CANdiSimState elevatorCandiSim;
    private CANrangeSimState elevatorRangeLeftSim;
    private CANrangeSimState elevatorRangeRightSim;

    public ElevatorSensorIOSim() {
        super();
        elevatorCandiSim = new CANdiSimState(elevatorCandi);
        elevatorRangeLeftSim = new CANrangeSimState(canRangeLeft);
        elevatorRangeRightSim = new CANrangeSimState(canRangeRight);

        setElevatorHallEffectTriggered();
    }

    @Override
    public void readInputs(ElevatorSensorInputs inputs) {
        super.readInputs(inputs);
    }

    public void setElevatorHallEffectUntriggered() {
        elevatorCandiSim.setS1State(S1StateValue.High);
    }

    public void setElevatorHallEffectTriggered() {
        elevatorCandiSim.setS1State(S1StateValue.Low);
    }

    public void setElevatorDistance(double leftM, double rightM) {
        elevatorRangeLeftSim.setDistance(leftM);
        elevatorRangeRightSim.setDistance(rightM);
    }
}
