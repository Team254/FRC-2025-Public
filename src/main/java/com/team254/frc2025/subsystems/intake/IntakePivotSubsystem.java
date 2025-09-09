package com.team254.frc2025.subsystems.intake;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.auto.AutoModeSelector.FeederStrategy;
import com.team254.lib.subsystems.*;
import com.team254.lib.util.Util;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code IntakePivotSubsystem} controls the pivoting mechanism of the robot's intake. It
 * manages the deployment and stowing of the intake using a motor with an integrated CANcoder for
 * precise position control and feedback.
 */
public class IntakePivotSubsystem
        extends ServoMotorSubsystemWithCanCoder<
                MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
    private final RobotState state;
    private boolean fullyDeployed = false;

    public IntakePivotSubsystem(
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

        this.setCurrentPosition(Constants.IntakeConstants.kIntakePivotStowPositionRadians);
        this.positionSetpointUnits = Constants.IntakeConstants.kIntakePivotStowPositionRadians;
        this.state = state;
    }

    public void setAutoDefaultCommand(FeederStrategy strategy) {
        this.getDefaultCommand().cancel();
        this.setDefaultCommand(
                motionMagicSetpointCommand(
                                () ->
                                        strategy == FeederStrategy.GROUND
                                                ? Constants.IntakeConstants
                                                        .kIntakePivotDeployPositionRadians
                                                : Constants.IntakeConstants
                                                        .kIntakePivotStowPositionRadians)
                        .withName("Intake Pivot Deploy in Auto (default)")
                        .ignoringDisable(true));
    }

    public void setTeleopDefaultCommand() {
        this.getDefaultCommand().cancel();
        this.setDefaultCommand(
                motionMagicSetpointCommand(this::getPositionSetpointUnits)
                        .withName("Intake Pivot Maintain Setpoint (default)")
                        .ignoringDisable(true));
    }

    public boolean hasCoralAtIntake() {
        return fullyDeployed
                && this.positionSetpointUnits
                        == Constants.IntakeConstants.kIntakePivotDeployPositionRadians
                && inputs.unitPosition
                        < Constants.IntakeConstants.kIntakePivotCoralPushedPositionRadians;
    }

    @Override
    public void periodic() {
        super.periodic();
        state.setIntakePivotRadians(inputs.unitPosition);
        if (!fullyDeployed)
            fullyDeployed =
                    inputs.unitPosition
                                    >= Constants.IntakeConstants.kIntakePivotDeployPositionRadians
                            || Util.epsilonEquals(
                                    Constants.IntakeConstants.kIntakePivotDeployPositionRadians,
                                    inputs.unitPosition,
                                    0.005);
        if (this.positionSetpointUnits
                != Constants.IntakeConstants.kIntakePivotDeployPositionRadians)
            fullyDeployed = false;
        Logger.recordOutput("IntakePivot/fullyDeployed", fullyDeployed);
    }
}
