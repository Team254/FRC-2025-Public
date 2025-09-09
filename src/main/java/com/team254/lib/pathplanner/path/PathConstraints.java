package com.team254.lib.pathplanner.path;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.json.simple.JSONObject;

/**
 * Kinematic path following constraints
 *
 * @param maxVelocityMPS Max linear velocity (M/S)
 * @param maxAccelerationMPSSq Max linear acceleration (M/S^2)
 * @param maxAngularVelocityRadPerSec Max angular velocity (Rad/S)
 * @param maxAngularAccelerationRadPerSecSq Max angular acceleration (Rad/S^2)
 * @param nominalVoltageVolts The nominal battery voltage (Volts)
 * @param unlimited Should the constraints be unlimited
 */
public record PathConstraints(
        double maxVelocityMPS,
        double maxAccelerationMPSSq,
        double maxAngularVelocityRadPerSec,
        double maxAngularAccelerationRadPerSecSq,
        double maxXAccelMPSSq,
        double maxYAccelMPSSq,
        double nominalVoltageVolts,
        boolean unlimited) {
    /**
     * Kinematic path following constraints
     *
     * @param maxVelocity Max linear velocity
     * @param maxAcceleration Max linear acceleration
     * @param maxAngularVelocity Max angular velocity
     * @param maxAngularAcceleration Max angular acceleration
     * @param nominalVoltage The nominal battery voltage
     * @param unlimited Should the constraints be unlimited
     */
    public PathConstraints(
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration,
            AngularVelocity maxAngularVelocity,
            AngularAcceleration maxAngularAcceleration,
            LinearAcceleration maxXAcceleration,
            LinearAcceleration maxYAcceleration,
            Voltage nominalVoltage,
            boolean unlimited) {
        this(
                maxVelocity.in(MetersPerSecond),
                maxAcceleration.in(MetersPerSecondPerSecond),
                maxAngularVelocity.in(RadiansPerSecond),
                maxAngularAcceleration.in(RadiansPerSecondPerSecond),
                maxXAcceleration.in(MetersPerSecondPerSecond),
                maxYAcceleration.in(MetersPerSecondPerSecond),
                nominalVoltage.in(Volts),
                unlimited);
    }

    /**
     * Kinematic path following constraints
     *
     * @param maxVelocityMPS Max linear velocity (M/S)
     * @param maxAccelerationMPSSq Max linear acceleration (M/S^2)
     * @param maxAngularVelocityRadPerSec Max angular velocity (Rad/S)
     * @param maxAngularAccelerationRadPerSecSq Max angular acceleration (Rad/S^2)
     * @param nominalVoltageVolts The nominal battery voltage (Volts)
     */
    public PathConstraints(
            double maxVelocityMPS,
            double maxAccelerationMPSSq,
            double maxAngularVelocityRadPerSec,
            double maxAngularAccelerationRadPerSecSq,
            double maxXAccelMPSSq,
            double maxYAccelMPSSq,
            double nominalVoltageVolts) {
        this(
                maxVelocityMPS,
                maxAccelerationMPSSq,
                maxAngularVelocityRadPerSec,
                maxAngularAccelerationRadPerSecSq,
                maxXAccelMPSSq,
                maxYAccelMPSSq,
                nominalVoltageVolts,
                false);
    }

    /**
     * Kinematic path following constraints
     *
     * @param maxVelocity Max linear velocity
     * @param maxAcceleration Max linear acceleration
     * @param maxAngularVelocity Max angular velocity
     * @param maxAngularAcceleration Max angular acceleration
     * @param nominalVoltage The nominal battery voltage
     */
    public PathConstraints(
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration,
            AngularVelocity maxAngularVelocity,
            AngularAcceleration maxAngularAcceleration,
            LinearAcceleration maxXAcceleration,
            LinearAcceleration maxYAcceleration,
            Voltage nominalVoltage) {
        this(
                maxVelocity,
                maxAcceleration,
                maxAngularVelocity,
                maxAngularAcceleration,
                maxXAcceleration,
                maxYAcceleration,
                nominalVoltage,
                false);
    }

    /**
     * Kinematic path following constraints
     *
     * @param maxVelocityMPS Max linear velocity (M/S)
     * @param maxAccelerationMPSSq Max linear acceleration (M/S^2)
     * @param maxAngularVelocityRadPerSec Max angular velocity (Rad/S)
     * @param maxAngularAccelerationRadPerSecSq Max angular acceleration (Rad/S^2)
     */
    public PathConstraints(
            double maxVelocityMPS,
            double maxAccelerationMPSSq,
            double maxAngularVelocityRadPerSec,
            double maxAngularAccelerationRadPerSecSq,
            double maxXAccelMPSSq,
            double maxYAccelMPSSq) {
        this(
                maxVelocityMPS,
                maxAccelerationMPSSq,
                maxAngularVelocityRadPerSec,
                maxAngularAccelerationRadPerSecSq,
                maxXAccelMPSSq,
                maxYAccelMPSSq,
                12.0,
                false);
    }

    /**
     * Kinematic path following constraints
     *
     * @param maxVelocity Max linear velocity
     * @param maxAcceleration Max linear acceleration
     * @param maxAngularVelocity Max angular velocity
     * @param maxAngularAcceleration Max angular acceleration
     */
    public PathConstraints(
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration,
            AngularVelocity maxAngularVelocity,
            AngularAcceleration maxAngularAcceleration,
            LinearAcceleration maxXAcceleration,
            LinearAcceleration maxYAcceleration) {
        this(
                maxVelocity,
                maxAcceleration,
                maxAngularVelocity,
                maxAngularAcceleration,
                maxXAcceleration,
                maxYAcceleration,
                Volts.of(12.0),
                false);
    }

    /**
     * Get unlimited PathConstraints
     *
     * @param nominalVoltage The nominal battery voltage (Volts)
     * @return Unlimited constraints
     */
    public static PathConstraints unlimitedConstraints(double nominalVoltage) {
        return new PathConstraints(
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                nominalVoltage,
                true);
    }

    /**
     * Create a path constraints object from json
     *
     * @param constraintsJson {@link org.json.simple.JSONObject} representing a path constraints
     *     object
     * @return The path constraints defined by the given json
     */
    static PathConstraints fromJson(JSONObject constraintsJson) {
        double maxVel = ((Number) constraintsJson.get("maxVelocity")).doubleValue();
        double maxAccel = ((Number) constraintsJson.get("maxAcceleration")).doubleValue();
        double maxAngularVel =
                ((Number) constraintsJson.get("maxAngularVelocity")).doubleValue(); // Degrees
        double maxAngularAccel =
                ((Number) constraintsJson.get("maxAngularAcceleration")).doubleValue(); // Degrees
        double nominalVoltage = ((Number) constraintsJson.get("nominalVoltage")).doubleValue();
        boolean unlimited = ((boolean) constraintsJson.get("unlimited"));

        return new PathConstraints(
                maxVel,
                maxAccel,
                Units.degreesToRadians(maxAngularVel),
                Units.degreesToRadians(maxAngularAccel),
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                nominalVoltage,
                unlimited);
    }

    /**
     * Get the max linear velocity
     *
     * @return Max linear velocity
     */
    public LinearVelocity maxVelocity() {
        return MetersPerSecond.of(maxVelocityMPS);
    }

    /**
     * Get the max linear acceleration
     *
     * @return Max linear acceleration
     */
    public LinearAcceleration maxAcceleration() {
        return MetersPerSecondPerSecond.of(maxAccelerationMPSSq);
    }

    /**
     * Get the max angular velocity
     *
     * @return Max angular velocity
     */
    public AngularVelocity maxAngularVelocity() {
        return RadiansPerSecond.of(maxAngularVelocityRadPerSec);
    }

    /**
     * Get the max angular acceleration
     *
     * @return Max angular acceleration
     */
    public AngularAcceleration maxAngularAcceleration() {
        return RadiansPerSecondPerSecond.of(maxAngularAccelerationRadPerSecSq);
    }

    public LinearAcceleration maxXAcceleration() {
        return MetersPerSecondPerSecond.of(maxXAccelMPSSq);
    }

    public LinearAcceleration maxYAcceleration() {
        return MetersPerSecondPerSecond.of(maxYAccelMPSSq);
    }
}
