package com.team254.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.team254.frc2025.Constants;
import com.team254.frc2025.Constants.SensorConstants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.subsystems.*;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code ElevatorSubsystem} controls the robot's elevator mechanism. It manages the vertical
 * movement of the elevator using motion profiling and integrates with sensors for accurate
 * positioning and safety features.
 */
public class ElevatorSubsystem
        extends ServoMotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {
    private final RobotState state;

    private ElevatorSensorInputsAutoLogged inputsSensors = new ElevatorSensorInputsAutoLogged();
    private ElevatorSensorIO ioSensors;
    private Trigger gyroCheckTrigger;

    public ElevatorSubsystem(
            ServoMotorSubsystemWithFollowersConfig leadConfig,
            MotorIO leadIo,
            MotorIO[] followerIo,
            final ElevatorSensorIO sensorIO,
            RobotState state) {
        super(
                leadConfig,
                new MotorInputsAutoLogged(),
                leadIo,
                new MotorInputsAutoLogged[] {new MotorInputsAutoLogged()},
                followerIo);
        this.state = state;
        this.ioSensors = sensorIO;

        gyroCheckTrigger =
                new Trigger(
                                () ->
                                        Math.abs(state.getDrivePitchRadians())
                                                        < Constants.DriveConstants
                                                                .kDrivePitchThresholdRadians
                                                && Math.abs(state.getDriveRollRadians())
                                                        < Constants.DriveConstants
                                                                .kDriveRollThresholdRadians)
                        .debounce(0.5);

        setCurrentPositionAsZero();
        setDefaultCommand(
                motionMagicSetpointCommand(this::getPositionSetpointUnits)
                        .withName(getName() + " Default Command Neutral")
                        .ignoringDisable(true));
    }

    @Override
    public void periodic() {
        super.periodic();

        ioSensors.readInputs(inputsSensors);
        Logger.processInputs("Elevator/sensors", inputsSensors);

        // Update robot state
        state.setElevatorHeightMeters(getCurrentPosition());
        Logger.recordOutput(getName() + "/positionMeters", getCurrentPosition());

        Logger.recordOutput(getName() + "/canStow", canStow());

        // In disabled, we can rezero if we see <0 position.
        if (DriverStation.isDisabled() && getCurrentPosition() < 0.0) {
            setCurrentPositionAsZero();
        }
    }

    @Override
    protected void setMotionMagicSetpointImpl(double units, int slot) {
        final double kSlowThreshold = 1.0;
        MotionMagicConfigs config;
        if (units > kSlowThreshold && this.getCurrentPosition() < kSlowThreshold) {
            config = Constants.kDefaultElevatorConfig;
        } else {
            config = Constants.kLowElevatorConfig;
        }
        super.setMotionMagicSetpointImpl(units, config, slot);
    }

    public boolean elevatorAtHallEffect() {
        return inputsSensors.elevatorHallEffectTriggered;
    }

    public boolean isStowed() {
        return getCurrentPosition() < Constants.ElevatorConstants.kElevatorToleranceMeters;
    }

    public boolean isAtLevel(SuperstructureState state) {
        return Util.epsilonEquals(
                getCurrentPosition(),
                state.getElevatorHeightMeters(),
                Constants.ElevatorConstants.kElevatorToleranceMeters);
    }

    final double kLookaheadS = 0.2;

    public boolean canStowElevator() {
        var speed = Math.max(0, state.getLatestRobotRelativeChassisSpeed().vxMetersPerSecond);
        return inputsSensors.canRangeDistanceLeft - speed * kLookaheadS
                        > SensorConstants.kCANRangeStowElevatorThresholdMeters
                && inputsSensors.canRangeDistanceRight - speed * kLookaheadS
                        > SensorConstants.kCANRangeStowElevatorThresholdMeters;
    }

    public boolean canStow() {
        var speed = Math.max(0, state.getLatestRobotRelativeChassisSpeed().vxMetersPerSecond);
        return inputsSensors.canRangeDistanceLeft - speed * kLookaheadS
                        > SensorConstants.kCANRangeStowThresholdMeters
                && inputsSensors.canRangeDistanceRight - speed * kLookaheadS
                        > SensorConstants.kCANRangeStowThresholdMeters;
    }

    public boolean gyroCheck() {
        return gyroCheckTrigger.getAsBoolean();
    }
}
