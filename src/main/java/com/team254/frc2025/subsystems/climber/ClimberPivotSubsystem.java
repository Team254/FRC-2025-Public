package com.team254.frc2025.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.frc2025.RobotState;
import com.team254.lib.subsystems.*;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Subsystem for controlling the climber pivot mechanism. Manages the motor control and position
 * feedback for the climber's pivot joint.
 */
public class ClimberPivotSubsystem
        extends ServoMotorSubsystemWithCanCoder<
                MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
    private RobotState state;
    private double rawPositionOffset;
    private AtomicBoolean climberDeployDone = new AtomicBoolean(false);

    public ClimberPivotSubsystem(
            final ServoMotorSubsystemWithCanCoderConfig motorConfig,
            final MotorIO motorIO,
            final CanCoderIO canCoderIO,
            RobotState state) {
        super(
                motorConfig,
                new MotorInputsAutoLogged(),
                motorIO,
                new CanCoderInputsAutoLogged(),
                canCoderIO);
        this.state = state;
        setDefaultCommand(
                motionMagicSetpointCommand(this::getPositionSetpointUnits)
                        .withName("Climber Pivot Maintain Setpoint (default)")
                        .ignoringDisable(true));
    }

    @Override
    public void periodic() {
        super.periodic();
        state.setClimberPivotRadians(inputs.unitPosition);
    }

    public void setNeutralMode(NeutralModeValue mode) {
        this.io.setNeutralMode(mode);
    }

    public void setForwardLimit(boolean enable) {
        io.setEnableHardLimits(enable, false);
    }

    public void setClimberDeployDone(boolean done) {
        climberDeployDone.set(done);
    }

    public AtomicBoolean getClimberDeployDone() {
        return climberDeployDone;
    }

    public double getRawRotorPosition() {
        return inputs.rawRotorPosition;
    }

    public void setRawPositionOffset(double offset) {
        rawPositionOffset = offset;
    }

    public double getRawPositionOffset() {
        return rawPositionOffset;
    }
}
