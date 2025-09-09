package com.team254.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A custom profiled PID controller that supports specifying an end velocity. */
public class CustomProfiledPIDController {
    private final PIDController m_controller;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_profile;

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double m_kV = 0.0;

    public CustomProfiledPIDController(
            double Kp,
            double Ki,
            double Kd,
            TrapezoidProfile.Constraints constraints,
            double period) {
        m_controller = new PIDController(Kp, Ki, Kd, period);
        m_constraints = constraints;

        m_profile = new TrapezoidProfile(constraints);
    }

    public CustomProfiledPIDController(
            double Kp,
            double Ki,
            double Kd,
            double Kv,
            TrapezoidProfile.Constraints constraints,
            double period) {
        this(Kp, Ki, Kd, constraints, period);
        m_kV = Kv;
    }

    public void setVelocityFF(double kV) {
        m_kV = kV;
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return m_constraints;
    }

    public void setGoal(double goalPosition, double goalVelocity) {
        m_goal = new TrapezoidProfile.State(goalPosition, goalVelocity);
    }

    public void setGoal(double goalPosition) {
        setGoal(goalPosition, 0.0);
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    public TrapezoidProfile.State getGoal() {
        return m_goal;
    }

    public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
    }

    public void reset(double measuredPosition, double measuredVelocity) {
        m_controller.reset();
        m_setpoint = new TrapezoidProfile.State(measuredPosition, measuredVelocity);
    }

    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }

    public double calculate(double measuredPosition, double measuredVelocity) {
        m_setpoint = m_profile.calculate(m_controller.getPeriod(), m_setpoint, m_goal);

        double positionOutput = m_controller.calculate(measuredPosition, m_setpoint.position);

        double velocityFF = m_kV * m_setpoint.velocity;

        return positionOutput + velocityFF;
    }

    public double calculate(
            double measuredPosition, double measuredVelocity, TrapezoidProfile.State goal) {
        setGoal(goal);
        return calculate(measuredPosition, measuredVelocity);
    }

    public boolean atGoal(double positionTolerance) {
        return Math.abs(m_goal.position - m_setpoint.position) < positionTolerance;
    }

    public PIDController getPIDController() {
        return m_controller;
    }
}
