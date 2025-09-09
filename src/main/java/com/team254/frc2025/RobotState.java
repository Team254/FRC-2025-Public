package com.team254.frc2025;

import com.team254.frc2025.controlboard.ModalControls;
import com.team254.frc2025.subsystems.led.LedState;
import com.team254.frc2025.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.lib.util.ConcurrentTimeInterpolatableBuffer;
import com.team254.lib.util.FieldConstants;
import com.team254.lib.util.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

/** Tracks robot state including pose, velocities, and mechanism positions. */
public class RobotState {

    public static final double LOOKBACK_TIME = 1.0;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);

        // Initialize mechanism positions
        elevatorHeightMeters.set(0.0);
        wristRadians.set(0.0);
        intakeRollerRotations.set(0.0);
        clawRollerRotations.set(0.0);
    }

    // State of robot.

    // Kinematic Frames
    // Robot's pose in field coordinates over time
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    // Current robot-relative chassis speeds (measured from encoders)
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    // Current field-relative chassis speeds (measured from encoders)
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    // Desired robot-relative chassis speeds (set by control systems)
    private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    // Desired field-relative chassis speeds (set by control systems)
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());

    private final AtomicInteger iteration = new AtomicInteger(0);

    private double lastUsedMegatagTimestamp = 0;
    private Pose2d lastUsedMegatagPose = Pose2d.kZero;
    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private double autoStartTime;

    private Optional<Pose2d> trajectoryTargetPose = Optional.empty();
    private Optional<Pose2d> trajectoryCurrentPose = Optional.empty();

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void incrementIterationCount() {
        iteration.incrementAndGet();
    }

    public int getIteration() {
        return iteration.get();
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void addDriveMotionMeasurements(
            double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredRobotRelativeChassisSpeeds,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    /**
     * Predicts robot's future pose based on current velocity.
     *
     * @param lookaheadTimeS How far ahead to predict (seconds)
     * @return Predicted pose
     */
    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot =
                maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot.exp(
                new Twist2d(
                        delta.vxMetersPerSecond,
                        delta.vyMetersPerSecond,
                        delta.omegaRadiansPerSecond));
    }

    /**
     * Like getPredictedFieldToRobot but caps negative velocities to zero. Used for non-holonomic
     * path planning.
     */
    public Pose2d getPredictedCappedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot =
                maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot.exp(
                new Twist2d(
                        Math.max(0.0, delta.vxMetersPerSecond),
                        Math.max(0.0, delta.vyMetersPerSecond),
                        delta.omegaRadiansPerSecond));
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
        return desiredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond =
                getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    public void setLedState(LedState state) {
        ledState.set(state);
    }

    public LedState getLedState() {
        return ledState.get();
    }

    private Optional<Double> getMaxAbsValueInRange(
            ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty()) return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(
            double minTime, double maxTime) {
        // Gyro yaw rate not set in sim.
        if (Robot.isReal()) return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(
            double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(
            double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = megatagEstimate.getTimestampSeconds();
        lastUsedMegatagPose = megatagEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public Pose2d lastUsedMegatagPose() {
        return lastUsedMegatagPose;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/YawAngularVelocity",
                    this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/RollAngularVelocity",
                    this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/PitchAngularVelocity",
                    this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchRads.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/PitchRads",
                    this.drivePitchRads.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollRads.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/RollRads",
                    this.driveRollRads.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/AccelX", this.accelX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/AccelY", this.accelY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput(
                "RobotState/DesiredChassisSpeedFieldFrame",
                getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput(
                "RobotState/DesiredChassisSpeedRobotFrame",
                getLatestDesiredRobotRelativeChassisSpeeds());
        Logger.recordOutput(
                "RobotState/MeasuredChassisSpeedFieldFrame",
                getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput(
                "RobotState/FusedChassisSpeedFieldFrame",
                getLatestFusedFieldRelativeChassisSpeed());

        // Add mechanism logging
        Logger.recordOutput("RobotState/ElevatorHeightMeters", getElevatorHeightMeters());
        Logger.recordOutput("RobotState/WristRadians", getWristRadians());
        Logger.recordOutput("RobotState/IntakeRollerRotations", getIntakeRollerRotations());
        Logger.recordOutput("RobotState/CoralRollerRotations", getClawRollerRotations());

        // Add LED state logging
        LedState currentLEDState = getLedState();
        Logger.recordOutput(
                "RobotState/LEDState",
                String.format(
                        "R:%d G:%d B:%d",
                        currentLEDState.red, currentLEDState.green, currentLEDState.blue));
    }

    private final AtomicReference<Optional<Integer>> exclusiveTag =
            new AtomicReference<>(Optional.empty());

    private final AtomicReference<Double> elevatorHeightMeters = new AtomicReference<>(0.0);
    private final AtomicReference<Double> wristRadians = new AtomicReference<>(0.0);
    private final AtomicReference<Double> clawRollerRotations = new AtomicReference<>(0.0);

    private final AtomicReference<Double> intakeRollerRotations = new AtomicReference<>(0.0);
    private final AtomicReference<Double> intakeRollerRPS = new AtomicReference<>(0.0);
    private final AtomicReference<Double> intakePivotRadians = new AtomicReference<>(0.0);

    private final AtomicReference<Double> indexerRotations = new AtomicReference<>(0.0);
    private final AtomicReference<Double> indexerRPS = new AtomicReference<>(0.0);

    private final AtomicReference<Double> climberPivotRadians = new AtomicReference<>(0.0);

    private final AtomicReference<Double> climberRollerRotations = new AtomicReference<>(0.0);

    private final AtomicReference<Double> clawRollerRPS = new AtomicReference<>(0.0);

    private final AtomicReference<LedState> ledState = new AtomicReference<>(LedState.kBlue);

    public void setClimberRollerRotations(double rotations) {
        climberRollerRotations.set(rotations);
    }

    public void setClimberPivotRadians(double radians) {
        climberPivotRadians.set(radians);
    }

    public void setIndexerRotations(double rotations) {
        indexerRotations.set(rotations);
    }

    public void setIndexerRPS(double rps) {
        indexerRPS.set(rps);
    }

    public double getIndexerRotations() {
        return indexerRotations.get();
    }

    public double getIndexerRPS() {
        return indexerRPS.get();
    }

    public void setIntakePivotRadians(double radians) {
        intakePivotRadians.set(radians);
    }

    public double getIntakePivotRadians() {
        return intakePivotRadians.get();
    }

    public void setElevatorHeightMeters(double heightMeters) {
        elevatorHeightMeters.set(heightMeters);
    }

    public void setWristRadians(double radians) {
        wristRadians.set(radians);
    }

    public void setIntakeRollerRotations(double rotations) {
        intakeRollerRotations.set(rotations);
    }

    public void setIntakeRollerRPS(double rps) {
        intakeRollerRPS.set(rps);
    }

    public void setClawRollerRotations(double rotations) {
        clawRollerRotations.set(rotations);
    }

    public double getElevatorHeightMeters() {
        return elevatorHeightMeters.get();
    }

    public double getWristRadians() {
        return wristRadians.get();
    }

    public double getIntakeRollerRotations() {
        return intakeRollerRotations.get();
    }

    public double getIntakeRollerRPS() {
        return intakeRollerRPS.get();
    }

    public double getClawRollerRotations() {
        return clawRollerRotations.get();
    }

    public double getClimberRollerRotations() {
        return climberRollerRotations.get();
    }

    public double getClimberPivotRadians() {
        return climberPivotRadians.get();
    }

    public void setClawRollerRPS(double rps) {
        clawRollerRPS.set(rps);
    }

    public double getClawRollerRPS() {
        return clawRollerRPS.get();
    }

    public void setExclusiveTag(int id) {
        exclusiveTag.set(Optional.of(id));
    }

    public void clearExclusiveTag() {
        exclusiveTag.set(Optional.empty());
    }

    public Optional<Integer> getExclusiveTag() {
        return exclusiveTag.get();
    }

    public void setTrajectoryTargetPose(Pose2d pose) {
        trajectoryTargetPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryTargetPose() {
        return trajectoryTargetPose;
    }

    public void setTrajectoryCurrentPose(Pose2d pose) {
        trajectoryCurrentPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryCurrentPose() {
        return trajectoryCurrentPose;
    }

    public double getDrivePitchRadians() {
        if (this.drivePitchRads.getInternalBuffer().lastEntry() != null) {
            return drivePitchRads.getInternalBuffer().lastEntry().getValue();
        }
        return 0.0;
    }

    public double getDriveRollRadians() {
        if (this.driveRollRads.getInternalBuffer().lastEntry() != null) {
            return driveRollRads.getInternalBuffer().lastEntry().getValue();
        }
        return 0.0;
    }

    public void logControllerMode() {
        Logger.recordOutput("Controller Mode", ModalControls.getInstance().getMode().toString());
    }

    public static boolean onOpponentSide(boolean isRedAlliance, Pose2d pose) {
        return (isRedAlliance
                        && pose.getTranslation().getX()
                                < FieldConstants.fieldLength / 2 - Constants.kMidlineBuffer)
                || (!isRedAlliance
                        && pose.getTranslation().getX()
                                > FieldConstants.fieldLength / 2 + Constants.kMidlineBuffer);
    }

    public boolean onOpponentSide() {
        return onOpponentSide(this.isRedAlliance(), this.getLatestFieldToRobot().getValue());
    }
}
