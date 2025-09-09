package com.team254.lib.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team254.lib.util.CANStatusLogger;
import com.team254.lib.util.CTREUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CanCoderIOHardware implements CanCoderIO {
    protected final CANcoder canCoder;
    protected CanCoderConfig config;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final BaseStatusSignal[] signals;
    double goodValues = 0.0;

    public CanCoderIOHardware(CanCoderConfig config) {
        this.config = config;

        canCoder = new CANcoder(config.CANID.getDeviceNumber(), config.CANID.getBus());

        CTREUtil.applyConfiguration(canCoder, this.config.config);
        positionSignal = canCoder.getAbsolutePosition();
        velocitySignal = canCoder.getVelocity();

        signals = new BaseStatusSignal[] {positionSignal, velocitySignal};

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals);

        CANStatusLogger.getInstance()
                .registerCANcoder(
                        "CANcoder_ID" + config.CANID.getDeviceNumber(),
                        canCoder,
                        config.CANID.getDeviceNumber(),
                        config.CANID.getBus());
    }

    @Override
    public void updateFrequency(double hz) {
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }

    @Override
    public void readInputs(CanCoderInputs inputs) {
        BaseStatusSignal.refreshAll(signals);
        if (Double.isNaN(inputs.absolutePositionRotations)) {
            BaseStatusSignal.waitForAll(10.0, positionSignal, velocitySignal);
            goodValues++;
        }
        if (goodValues < 50) return;

        inputs.absolutePositionRotations = positionSignal.getValue().in(Rotations);
        inputs.velocityRotations = velocitySignal.getValue().in(RotationsPerSecond);
    }
}
