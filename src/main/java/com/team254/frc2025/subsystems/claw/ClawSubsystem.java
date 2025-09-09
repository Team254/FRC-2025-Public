package com.team254.frc2025.subsystems.claw;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.controlboard.ModalControls.Mode;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code ClawSubsystem} controls the claw mechanism, including intake and scoring of game
 * pieces. It integrates with both motors and sensor inputs, using banner sensors to detect the
 * presence of game pieces and manage the claw's operational states.
 */
public class ClawSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final RobotContainer robotContainer;
    private final RobotState robotState;
    private final CoralStateTracker coralStateTracker;
    protected TorqueCurrentFOC torqueControlIntake;
    protected TorqueCurrentFOC torqueControlHold;
    protected TorqueCurrentFOC torqueControlScore;
    protected TorqueCurrentFOC torqueControlNeutral;
    protected Trigger hasAlgaeTrigger;
    protected double wristRPS;
    protected double lastAlgaeScoreTimestamp;
    protected double lastHasAlgaeUndebouncedTimestamp;
    protected double lastAutoTimestamp;
    protected Mode currentMode;

    public enum VoltageMode {
        DEFAULT,
        L3_L2
    }

    private ClawSensorInputsAutoLogged inputsSensors = new ClawSensorInputsAutoLogged();
    private ClawSensorIO ioSensors;
    private Trigger hasCoralAtStageBannerTrigger;
    private VoltageMode voltageMode = VoltageMode.DEFAULT;
    private VoltageMode wantedVoltageMode = VoltageMode.DEFAULT;
    private boolean enableAutoScore;

    public ClawSubsystem(
            final ServoMotorSubsystemConfig motorConfig,
            final MotorIO motorIO,
            final ClawSensorIO sensorIO,
            RobotContainer robotContainer) {
        super(motorConfig, new MotorInputsAutoLogged(), motorIO);

        // Coral
        this.coralStateTracker = robotContainer.getCoralStateTracker();
        this.robotState = robotContainer.getRobotState();
        this.robotContainer = robotContainer;
        this.ioSensors = sensorIO;
        setCurrentPositionAsZero();
        enableAutoScore = false;
        hasCoralAtStageBannerTrigger =
                new Trigger(() -> inputsSensors.stageCoralBannerTriggered)
                        .debounce(
                                Constants.SensorConstants.kCoralStageBannerDebounceTime,
                                DebounceType.kBoth);

        // Algae
        lastAlgaeScoreTimestamp = 0.0;
        lastHasAlgaeUndebouncedTimestamp = 0.0;
        lastAutoTimestamp = 0.0;
        DoubleSupplier torqueControlSupplier =
                () -> {
                    if (hasAlgae()
                            && Math.abs(wristRPS)
                                    > Constants.ClawConstants.kAlgaeRollerThresholdVelocityRPS
                            && RobotTime.getTimestampSeconds() - lastAlgaeScoreTimestamp > 1) {
                        return Constants.ClawConstants.kAlgaeRollerTorqueCurrentIntake;
                    }
                    if (hasAlgae()
                            && RobotTime.getTimestampSeconds() - lastAlgaeScoreTimestamp > 1) {
                        return Constants.ClawConstants.kAlgaeRollerTorqueCurrentHold;
                    } else {
                        return 0.0;
                    }
                };

        hasAlgaeTrigger = new Trigger(() -> hasAlgae());
        BooleanSupplier hasAlgaeSupplier = () -> hasAlgaeTrigger.getAsBoolean();

        setDefaultCommand(
                new DefaultCommand(torqueControlSupplier, hasAlgaeSupplier).ignoringDisable(true));
    }

    public double getPositionRotations() {
        return inputs.unitPosition;
    }

    private double lastTimeAlgaeStateChanged = 0.0;
    private boolean lastAlgaeState = false;
    private boolean hasAlgaeDebounced = false;
    private boolean firstVoltageHack = false;

    public void setWantedVoltageMode(VoltageMode mode) {
        wantedVoltageMode = mode;
    }

    public boolean hasAlgae() {
        boolean current = hasAlgaeUndebounced();
        double currentTime = RobotTime.getTimestampSeconds();
        double debounceTime =
                DriverStation.isAutonomous()
                        ? 0.1
                        : Constants.ClawConstants.kAlgaeRollerDebounceTime;
        if (current != lastAlgaeState) {
            lastTimeAlgaeStateChanged = currentTime;
            lastAlgaeState = current;
        }
        if (currentTime - lastTimeAlgaeStateChanged >= debounceTime) {
            hasAlgaeDebounced = current;
        }
        Logger.recordOutput("Claw/debounceTime", debounceTime);
        return hasAlgaeDebounced;
    }

    @Override
    public void periodic() {
        super.periodic();
        robotState.setClawRollerRotations(getPositionRotations());
        robotState.setClawRollerRPS(getCurrentVelocity());
        coralStateTracker.updateStageBanner(hasCoralAtStageBanner());
        ioSensors.readInputs(inputsSensors);
        Logger.processInputs(getName() + "/sensors", inputsSensors);
        Logger.recordOutput(getName() + "/hasAlgae", hasAlgae());
        Logger.recordOutput(getName() + "/hasAlgaeUndebounced", hasAlgaeUndebounced());
        Logger.recordOutput("Last Algae Score Timestamp", lastAlgaeScoreTimestamp);

        if (hasAlgaeUndebounced()) {
            lastHasAlgaeUndebouncedTimestamp = RobotTime.getTimestampSeconds();
        }

        if (DriverStation.isAutonomous()) {
            lastAutoTimestamp = RobotTime.getTimestampSeconds();
        }

        if (wantedVoltageMode != voltageMode) {
            Logger.recordOutput(getName() + "/Last_Voltage_Swap_Time", Timer.getFPGATimestamp());
            if (wantedVoltageMode == VoltageMode.DEFAULT) {
                io.setVoltageConfig(Constants.kDefaultClawVoltageConfigs);
            } else {
                if (!firstVoltageHack) {
                    firstVoltageHack = true;
                    Constants.kL3_L2ClawVoltageConfigs.PeakForwardVoltage = 5.5;
                    Logger.recordOutput(getName() + "/hack", true);
                } else {
                    Constants.kL3_L2ClawVoltageConfigs.PeakForwardVoltage = 6;
                    Logger.recordOutput(getName() + "/hack", false);
                }
                io.setVoltageConfig(Constants.kL3_L2ClawVoltageConfigs);
            }
            voltageMode = wantedVoltageMode;
        }
        Logger.recordOutput(getName() + "/VoltageMode", voltageMode);
    }

    public void setAutoScore(boolean autoScore) {
        enableAutoScore = autoScore;
        if (autoScore) {
            setPositionSetpointImpl(0.0);
        }
    }

    public boolean autoScoreEnabled() {
        return enableAutoScore;
    }

    public void setAutoStage(boolean autoStage) {
        io.setEnableHardLimits(autoStage, false);
        io.setEnableAutosetPositionValue(false, false);
    }

    /**
     * @return True if coral is detected at the scoring banner sensor
     */
    public boolean hasCoralAtScoreBanner() {
        return inputsSensors.scoreCoralBannerTriggered;
    }

    /**
     * @return True if coral is detected at the staging banner sensor
     */
    public boolean hasCoralAtStageBanner() {
        return hasCoralAtStageBannerTrigger.getAsBoolean();
    }

    /**
     * Checks for algae detection without debouncing. This is the raw sensor reading.
     *
     * @return True if algae is detected by the sensor
     */
    public boolean hasAlgaeUndebounced() {
        // Detect algae when current is high but velocity is low
        return inputs.currentStatorAmps > 20
                && currentMode == Mode.ALGAECLIMB
                // only possible coral states in algae mode with an algae
                && (coralStateTracker.getCurrentPosition() == CoralStateTracker.CoralPosition.NONE
                        || coralStateTracker.getCurrentPosition()
                                == CoralStateTracker.CoralPosition.AT_FIRST_INDEXER);
    }

    public void updateWristRPS(double wristRPS) {
        this.wristRPS = wristRPS;
    }

    public void updateMode(Mode currentMode) {
        this.currentMode = currentMode;
    }

    public void updateLastAlgaeScoreTimestamp(double timestamp) {
        this.lastAlgaeScoreTimestamp = timestamp;
    }

    public class DefaultCommand extends Command {
        private final DoubleSupplier torqueControlSupplier;
        private final BooleanSupplier hasAlgaeSupplier;
        private boolean lastHasAlgae;

        public DefaultCommand(
                DoubleSupplier torqueControlSupplier, BooleanSupplier hasAlgaeSupplier) {
            this.torqueControlSupplier = torqueControlSupplier;
            this.hasAlgaeSupplier = hasAlgaeSupplier;
            addRequirements(ClawSubsystem.this);
        }

        @Override
        public void initialize() {
            lastHasAlgae = true;
        }

        @Override
        public void execute() {
            // Attempt to regrab algae after disabled auto if within time window
            if (RobotTime.getTimestampSeconds() - lastHasAlgaeUndebouncedTimestamp
                            < Constants.ClawConstants.kAlgaeRollerRegrabTimeSeconds
                    && RobotTime.getTimestampSeconds() - lastAutoTimestamp
                            < Constants.ClawConstants.kAlgaeRollerRegrabTimeSeconds) {
                setTorqueCurrentFOCImpl(Constants.ClawConstants.kAlgaeRollerTorqueCurrentIntake);
                Logger.recordOutput("ClawDefaultCommand/Latest Claw Command", "regrab algae");
                lastHasAlgae = true;
            } else if (hasAlgaeSupplier.getAsBoolean() && currentMode == Mode.ALGAECLIMB) {
                setTorqueCurrentFOCImpl(torqueControlSupplier.getAsDouble());
                Logger.recordOutput("ClawDefaultCommand/Latest Claw Command", "algae");
                lastHasAlgae = true;
            } else {
                if (lastHasAlgae) {
                    setCurrentPositionAsZero();
                    lastHasAlgae = false;
                    Logger.recordOutput(
                            "Last_Position_Reset_Timestamp", RobotTime.getTimestampSeconds());
                }
                if (enableAutoScore) {
                    setPositionSetpointImpl(0.0);
                }
                if (!robotContainer.getWrist().clearOfIndexer()) {
                    setVoltageImpl(0.0);
                }
                Logger.recordOutput("ClawDefaultCommand/Latest Claw Command", "coral");
            }
            Logger.recordOutput("Claw/enableAutoScore", enableAutoScore);
        }
    }
}
