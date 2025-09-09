package com.team254.frc2025.subsystems.climber;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the climber rollers. Manages the motor control and sensor feedback for
 * the climber mechanism.
 */
public class ClimberRollerSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

    private ClimberSensorInputsAutoLogged inputsSensors = new ClimberSensorInputsAutoLogged();
    private ClimberSensorIO ioSensors;
    private RobotState state;
    private Trigger climberHallEffectTrigger;
    private Trigger climberLeftLimitSwitchTrigger;
    private Trigger climberRightLimitSwitchTrigger;

    public ClimberRollerSubsystem(
            final ServoMotorSubsystemConfig motorConfig,
            final MotorIO motorIO,
            final ClimberSensorIO sensorIO,
            RobotState state) {
        super(motorConfig, new MotorInputsAutoLogged(), motorIO);
        this.ioSensors = sensorIO;
        this.state = state;

        climberLeftLimitSwitchTrigger =
                new Trigger(() -> inputsSensors.climberLeftLimitSwitchTriggered)
                        .debounce(Constants.SensorConstants.kClimberLatchDebounceTime);
        climberRightLimitSwitchTrigger =
                new Trigger(() -> inputsSensors.climberRightLimitSwitchTriggered)
                        .debounce(Constants.SensorConstants.kClimberLatchDebounceTime);
    }

    @Override
    public void periodic() {
        super.periodic();
        ioSensors.readInputs(inputsSensors);
        Logger.processInputs(getName() + "/sensors", inputsSensors);
        this.state.setClimberRollerRotations(inputs.unitPosition);
    }

    public boolean climberHallEffectTriggered() {
        return climberHallEffectTrigger.getAsBoolean();
    }

    public boolean climberLeftLimitSwitchTriggered() {
        return climberLeftLimitSwitchTrigger.getAsBoolean();
    }

    public boolean climberRightLimitSwitchTriggered() {
        return climberRightLimitSwitchTrigger.getAsBoolean();
    }
}
