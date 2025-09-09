package com.team254.lib.util;

import java.util.OptionalDouble;

public class ExponentialMovingAverage {
    private final double alpha;
    private OptionalDouble oldValue = OptionalDouble.empty();

    public ExponentialMovingAverage(double alpha) {
        this.alpha = alpha;
    }

    public double calculate(double value) {
        if (oldValue.isEmpty()) {
            oldValue = OptionalDouble.of(value);
            return value;
        }
        double newValue = oldValue.getAsDouble() + alpha * (value - oldValue.getAsDouble());
        oldValue = OptionalDouble.of(newValue);
        return newValue;
    }

    public void reset() {
        oldValue = OptionalDouble.empty();
    }
}
