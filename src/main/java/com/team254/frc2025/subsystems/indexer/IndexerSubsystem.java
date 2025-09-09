package com.team254.frc2025.subsystems.indexer;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code IndexerSubsystem} controls the robot's indexer mechanism. It manages the movement of
 * game pieces through the indexer using banner sensors to detect their presence and position.
 */
public class IndexerSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final RobotState state;
    private final CoralStateTracker coralStateTracker;

    private IndexerSensorInputsAutoLogged inputsSensors = new IndexerSensorInputsAutoLogged();
    private IndexerSensorIO ioSensors;
    private Trigger hasCoralAtFirstIndexerBannerTrigger;
    private Trigger hasCoralAtSecondIndexerBannerTrigger;

    public IndexerSubsystem(
            final ServoMotorSubsystemConfig motorConfig,
            final MotorIO motorIO,
            final IndexerSensorIO sensorIO,
            RobotState state,
            CoralStateTracker coralStateTracker) {
        super(motorConfig, new MotorInputsAutoLogged(), motorIO);
        this.coralStateTracker = coralStateTracker;
        this.state = state;
        this.ioSensors = sensorIO;

        hasCoralAtFirstIndexerBannerTrigger =
                new Trigger(() -> inputsSensors.firstIndexerBannerTriggered)
                        .debounce(
                                Constants.SensorConstants.kFirstIndexerBannerDebounceTime,
                                DebounceType.kBoth);
        hasCoralAtSecondIndexerBannerTrigger =
                new Trigger(() -> inputsSensors.secondIndexerBannerTriggered)
                        .debounce(
                                Constants.SensorConstants.kSecondIndexerBannerDebounceTime,
                                DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        super.periodic();
        state.setIndexerRotations(inputs.unitPosition);
        state.setIndexerRPS(inputs.velocityUnitsPerSecond);
        coralStateTracker.updateFirstIndexer(hasCoralAtFirstIndexerBanner());
        coralStateTracker.updateSecondIndexer(hasCoralAtSecondIndexerBanner());
        ioSensors.readInputs(inputsSensors);
        Logger.processInputs("Indexer/sensors", inputsSensors);
    }

    public boolean hasCoralAtFirstIndexerBanner() {
        return hasCoralAtFirstIndexerBannerTrigger.getAsBoolean();
    }

    public boolean hasCoralAtSecondIndexerBanner() {
        return hasCoralAtSecondIndexerBannerTrigger.getAsBoolean();
    }

    public void setForwardLimit(boolean enable) {
        io.setEnableHardLimits(enable, false);
    }
}
