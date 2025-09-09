package com.team254.lib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class CTREUtil {
    public static final int MAX_RETRIES = 10;

    public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        final int max_num_retries = 10;
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < max_num_retries; ++i) {
            statusCode = function.get();
            if (statusCode == StatusCode.OK) break;
        }
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                    "Error calling "
                            + function
                            + " on ctre device id "
                            + deviceId
                            + ": "
                            + statusCode,
                    true);
        }
        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, VoltageConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfigurationNonBlocking(TalonFX motor, VoltageConfigs config) {
        return motor.getConfigurator().apply(config, 0.01);
    }

    public static StatusCode applyConfiguration(TalonFX motor, HardwareLimitSwitchConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, MotionMagicConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static CANdiConfiguration createCandiConfiguration() {
        CANdiConfiguration candiConfiguration = new CANdiConfiguration();
        candiConfiguration.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        candiConfiguration.DigitalInputs.S1FloatState = S1FloatStateValue.PullLow;
        candiConfiguration.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotHigh;
        candiConfiguration.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;
        return candiConfiguration;
    }

    public static CANdiConfiguration createCustomCandiConfiguration(
            S1CloseStateValue s1CloseState,
            S1FloatStateValue s1FloatState,
            S2CloseStateValue s2CloseState,
            S2FloatStateValue s2FloatState) {
        CANdiConfiguration candiConfiguration = new CANdiConfiguration();
        candiConfiguration.DigitalInputs.S1CloseState = s1CloseState;
        candiConfiguration.DigitalInputs.S1FloatState = s1FloatState;
        candiConfiguration.DigitalInputs.S2CloseState = s2CloseState;
        candiConfiguration.DigitalInputs.S2FloatState = s2FloatState;
        return candiConfiguration;
    }
}
