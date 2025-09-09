package com.team254.lib.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.team254.lib.drivers.CANDeviceId;

public class CanCoderConfig {
    public CANDeviceId CANID;
    public CANcoderConfiguration config = new CANcoderConfiguration();
}
