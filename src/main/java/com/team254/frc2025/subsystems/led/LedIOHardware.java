package com.team254.frc2025.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.team254.frc2025.Constants;
import com.team254.frc2025.Robot;

public class LedIOHardware implements LedIO {
    private final CANdle candle;
    private LedState currentState = LedState.kBlue;
    private LedState[] currentPixels =
            new LedState
                    [Constants.LEDConstants.kCandleLEDCount
                            + Constants.LEDConstants.kNonCandleLEDCount];

    public LedIOHardware() {
        if (Robot.isReal()) {
            candle =
                    new CANdle(
                            Constants.LEDConstants.kCANdleId.getDeviceNumber(),
                            Constants.LEDConstants.kCANdleId.getBus());
            candle.configBrightnessScalar(1.0);
            candle.configLEDType(CANdle.LEDStripType.RGB);
        } else {
            candle = null;
        }
    }

    public LedState getCurrentState() {
        return currentState;
    }

    public LedState[] getCurrentPixels() {
        return currentPixels;
    }

    @Override
    public void writePixels(LedState state) {
        if (state == null) state = LedState.kOff;
        currentState = state;
        if (candle != null) candle.setLEDs(state.red, state.green, state.blue);
    }

    @Override
    public void writePixels(LedState[] pixels) {
        // do not write empty data
        if (pixels == null || pixels.length == 0) {
            return;
        }

        LedState run = pixels[0];
        int runStart = 0;
        for (int i = 0; i < pixels.length; i++) {
            if (pixels[i] == null) pixels[i] = LedState.kOff;
            if (!run.equals(pixels[i])) {
                if (candle != null)
                    candle.setLEDs(run.red, run.green, run.blue, 255, runStart, i - runStart);
                runStart = i;
                run = pixels[i];
                currentPixels[i] = run;
            }
        }

        if (candle != null)
            candle.setLEDs(run.red, run.green, run.blue, 255, runStart, pixels.length - runStart);
    }
}
