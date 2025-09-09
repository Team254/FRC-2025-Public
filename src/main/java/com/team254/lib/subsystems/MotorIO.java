package com.team254.lib.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.drivers.CANDeviceId;

public interface MotorIO {
    void readInputs(MotorInputs inputs);

    void setOpenLoopDutyCycle(double dutyCycle);

    // These are in the "units" of the subsystem (rad, m).
    void setPositionSetpoint(double units);

    default void setMotionMagicSetpoint(double units) {
        setMotionMagicSetpoint(units, 0);
    }

    void setMotionMagicSetpoint(double units, int slot);

    default void setMotionMagicSetpoint(
            double units, double velocity, double acceleration, double jerk) {
        setMotionMagicSetpoint(units, velocity, acceleration, jerk, 0);
    }

    default void setMotionMagicSetpoint(
            double units, double velocity, double acceleration, double jerk, int slot) {
        setMotionMagicSetpoint(units, velocity, acceleration, jerk, slot, 0.0);
    }

    void setMotionMagicSetpoint(
            double units,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedforward);

    void setNeutralMode(NeutralModeValue mode);

    default void setVelocitySetpoint(double unitsPerSecond) {
        setVelocitySetpoint(unitsPerSecond, 0);
    }

    void setVelocitySetpoint(double unitsPerSecond, int slot);

    void setVoltageOutput(double voltage);

    void setCurrentPositionAsZero();

    void setCurrentPosition(double positionUnits);

    void setEnableSoftLimits(boolean forward, boolean reverse);

    void setEnableHardLimits(boolean forward, boolean reverse);

    void setEnableAutosetPositionValue(boolean forward, boolean reverse);

    void follow(CANDeviceId masterId, boolean opposeMasterDirection);

    void setTorqueCurrentFOC(double current);

    void setMotionMagicConfig(MotionMagicConfigs config);

    void setVoltageConfig(VoltageConfigs config);
}
