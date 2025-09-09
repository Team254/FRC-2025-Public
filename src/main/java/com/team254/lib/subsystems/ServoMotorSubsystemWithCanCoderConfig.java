package com.team254.lib.subsystems;

public class ServoMotorSubsystemWithCanCoderConfig extends ServoMotorSubsystemConfig {
    public CanCoderConfig canCoderConfig = new CanCoderConfig();

    // This is the ratio from cancoder to units.
    // cancoder rotations * by this ratio should = units of subsystem.
    public double cancoderToUnitsRatio = 1.0;

    // This is the ratio from rotor to cancoder.
    // rotor * by this ratio should = cancoder.
    public double getCanCodertoRotorRatio() {
        return unitToRotorRatio / cancoderToUnitsRatio;
    }

    public boolean isFusedCancoder = false;
    public double ratioForSim = 1.0;
    public double cancoderUnitsForSim = 1.0;
}
