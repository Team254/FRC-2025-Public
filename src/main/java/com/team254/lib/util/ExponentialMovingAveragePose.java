package com.team254.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ExponentialMovingAveragePose {
    private final ExponentialMovingAverage filterX;
    private final ExponentialMovingAverage filterY;
    private final ExponentialMovingAverage filterTheta;

    public ExponentialMovingAveragePose(double alpha) {
        filterX = new ExponentialMovingAverage(alpha);
        filterY = new ExponentialMovingAverage(alpha);
        filterTheta = new ExponentialMovingAverage(alpha);
    }

    public void reset() {
        filterX.reset();
        filterY.reset();
        filterTheta.reset();
    }

    public Pose2d calculate(Pose2d value) {
        double newX = filterX.calculate(value.getX());
        double newY = filterY.calculate(value.getY());
        double newTheta = filterTheta.calculate(value.getRotation().getRadians());
        return new Pose2d(newX, newY, new Rotation2d(newTheta));
    }
}
