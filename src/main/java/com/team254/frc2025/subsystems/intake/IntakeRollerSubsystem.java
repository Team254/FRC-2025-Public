package com.team254.frc2025.subsystems.intake;

import com.team254.frc2025.RobotState;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;

/**
 * The {@code IntakeRollerSubsystem} controls the roller mechanism of the robot's intake. It manages
 * the speed and direction of the intake rollers to collect and feed game pieces.
 */
public class IntakeRollerSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final RobotState state;
    public MotorIO motorIO;

    public IntakeRollerSubsystem(
            final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO, RobotState state) {
        super(motorConfig, new MotorInputsAutoLogged(), motorIO);
        this.state = state;
        this.motorIO = motorIO;
    }

    public double getPositionRotations() {
        return inputs.unitPosition;
    }

    @Override
    public void periodic() {
        super.periodic();
        state.setIntakeRollerRotations(getPositionRotations());
        state.setIntakeRollerRPS(getCurrentVelocity());
    }
}
