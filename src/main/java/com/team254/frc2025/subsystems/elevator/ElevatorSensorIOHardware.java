package com.team254.frc2025.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.team254.frc2025.Constants;
import com.team254.lib.util.CTREUtil;
import edu.wpi.first.units.measure.Distance;

/**
 * The {@code ElevatorSensorIOHardware} class implements the {@link ElevatorSensorIO} interface to
 * provide hardware-level access to elevator sensors, including hall effect sensors and CAN range
 * finders. It handles the configuration and reading of these sensors.
 */
public class ElevatorSensorIOHardware implements ElevatorSensorIO {

    protected final CANdi elevatorCandi;
    protected final CANrange canRangeLeft;
    protected final CANrange canRangeRight;
    protected final StatusSignal<Boolean> elevatorHallEffectTriggered;
    protected final StatusSignal<Distance> canRangeDistanceLeft;
    protected final StatusSignal<Distance> canRangeDistanceRight;
    protected final StatusSignal<Double> canRangeStrengthLeft;
    protected final StatusSignal<Double> canRangeStrengthRight;

    public ElevatorSensorIOHardware() {
        elevatorCandi =
                new CANdi(
                        Constants.SensorConstants.kElevatorCandiID,
                        Constants.kCanBusSuperstructureCanivore);
        elevatorCandi.getConfigurator().apply(CTREUtil.createCandiConfiguration());

        canRangeLeft =
                new CANrange(
                        Constants.SensorConstants.kCANrangeDeviceLeftId,
                        Constants.kCanBusDrivebaseClimberCanivore);
        canRangeRight =
                new CANrange(
                        Constants.SensorConstants.kCANrangeDeviceRightId,
                        Constants.kCanBusDrivebaseClimberCanivore);

        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        canRangeConfig.FovParams.FOVRangeX = 6.75;
        canRangeConfig.FovParams.FOVRangeY = 6.75;

        canRangeLeft.getConfigurator().apply(canRangeConfig);
        canRangeRight.getConfigurator().apply(canRangeConfig);

        elevatorHallEffectTriggered = elevatorCandi.getS1Closed();
        canRangeDistanceLeft = canRangeLeft.getDistance();
        canRangeStrengthLeft = canRangeLeft.getSignalStrength();

        canRangeDistanceRight = canRangeRight.getDistance();
        canRangeStrengthRight = canRangeRight.getSignalStrength();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SensorConstants.kElevatorRefreshRateHz,
                elevatorHallEffectTriggered,
                canRangeStrengthLeft,
                canRangeStrengthRight,
                canRangeDistanceLeft,
                canRangeDistanceRight);
    }

    @Override
    public void readInputs(ElevatorSensorInputs inputs) {
        BaseStatusSignal.refreshAll(elevatorHallEffectTriggered);

        BaseStatusSignal.refreshAll(
                canRangeDistanceLeft,
                canRangeDistanceRight,
                canRangeStrengthLeft,
                canRangeStrengthRight);

        inputs.elevatorHallEffectTriggered = elevatorHallEffectTriggered.getValue();

        double threshold = 1500.0;

        inputs.canRangeSignalStrengthLeft = canRangeStrengthLeft.getValue();
        if (inputs.canRangeSignalStrengthLeft > threshold) {
            inputs.canRangeDistanceLeft =
                    canRangeDistanceLeft.getValue().in(edu.wpi.first.units.Units.Meters);
        } else {
            inputs.canRangeDistanceLeft = Double.POSITIVE_INFINITY;
        }

        inputs.canRangeSignalStrengthRight = canRangeStrengthRight.getValue();
        if (inputs.canRangeSignalStrengthRight > threshold) {
            inputs.canRangeDistanceRight =
                    canRangeDistanceRight.getValue().in(edu.wpi.first.units.Units.Meters);
        } else {
            inputs.canRangeDistanceRight = Double.POSITIVE_INFINITY;
        }
    }
}
