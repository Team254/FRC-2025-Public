package com.team254.frc2025;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.team254.frc2025.subsystems.drive.CommandSwerveDrivetrain;
import com.team254.frc2025.subsystems.drive.CompTunerConstants;
import com.team254.frc2025.subsystems.drive.PracTunerConstants;
import com.team254.frc2025.subsystems.drive.SimTunerConstants;
import com.team254.lib.drivers.CANDeviceId;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.subsystems.ServoMotorSubsystemWithCanCoderConfig;
import com.team254.lib.subsystems.ServoMotorSubsystemWithFollowersConfig;
import com.team254.lib.subsystems.SimElevator;
import com.team254.lib.util.FieldConstants.Reef;
import com.team254.lib.util.FieldConstants.ReefHeight;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final SimControllerType kSimControllerType = SimControllerType.XBOX;

    public enum SimControllerType {
        XBOX,
        DUAL_SENSE
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final String kCanBusDrivebaseClimberCanivore = "drivebase-climber";
    public static final String kCanBusSuperstructureCanivore = "superstructure";
    public static boolean kIsReplay = false;
    public static final String kPracticeBotMacAddress = "00:80:2F:33:BF:BB";
    public static boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

    public static final double kSteerJoystickDeadband = 0.05;
    public static final double kRobotWidth = Units.inchesToMeters(35.625);
    public static final double kRobotDiagonal = Math.sqrt(2.0) * kRobotWidth;
    public static final double kRobotMassKg = Units.lbsToKilograms(147.92);
    public static final double kRobotMomentOfInertia = 2 * 9.38; // kg * m^2
    public static final double kCOGHeightMeters = Units.inchesToMeters(0.0);

    public static final ClosedLoopRampsConfigs makeDefaultClosedLoopRampConfig() {
        return new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(0.02)
                .withTorqueClosedLoopRampPeriod(0.02)
                .withVoltageClosedLoopRampPeriod(0.02);
    }

    public static final OpenLoopRampsConfigs makeDefaultOpenLoopRampConfig() {
        return new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.02)
                .withTorqueOpenLoopRampPeriod(0.02)
                .withVoltageOpenLoopRampPeriod(0.02);
    }

    public static final class DriveConstants {
        public static final Transform2d kDriveToCoralOffset =
                new Transform2d(new Translation2d(0.66, 0.0), Rotation2d.kZero);
        public static final double kDriveMaxSpeed = 3.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
        public static final double kMaxXAccelerationMetersPerSecondSquared = 10.0;
        public static final double kMaxYAccelerationMetersPerSecondSquared = 10.0;
        public static final double kDriveMaxAngularRate = 8.2;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
        public static final double kHeadingControllerP = 5.0;
        public static final double kHeadingControllerI = 0;
        public static final double kHeadingControllerD = 0;
        public static final CommandSwerveDrivetrain kDrivetrain =
                Robot.isSimulation()
                        ? SimTunerConstants.createDrivetrain()
                        : (kIsPracticeBot
                                ? PracTunerConstants.createDrivetrain()
                                : CompTunerConstants.createDrivetrain());
        public static final double kRobotWeightPounds = 150.0;
        public static final double kBumperLengthInches = 35.625;
        public static final double kBumperWidthInches = 35.625;
        public static final double kWheelCoefficientOfFriction = 1.0;
        public static final int kDriveMotorCount = 1;

        public static final double kDisabledDriveXStdDev = 1.0;
        public static final double kDisabledDriveYStdDev = 1.0;
        public static final double kDisabledDriveRotStdDev = 1.0;

        public static final double kEnabledDriveXStdDev = 0.3;
        public static final double kEnabledDriveYStdDev = 0.3;
        public static final double kEnabledDriveRotStdDev = 0.2;

        public static final double kDrivePitchThresholdRadians = Units.degreesToRadians(10.0);
        public static final double kDriveRollThresholdRadians = Units.degreesToRadians(10.0);
    }

    public static final double kJoystickThreshold = 0.1;
    public static final int kDriveGamepadPort = 0;
    public static final boolean useMapleSim = true;
    public static final double kPOVDebounceTimeSeconds = 0.1;

    public static final class SensorConstants {
        public static final int kClawCoralCandiID = 33;
        public static final int kClimberLimitSwitchCandiID = 17;
        public static final int kIndexerCandiID = 31;
        public static final int kElevatorCandiID = 32;

        public static final int kCANrangeDeviceLeftId = 41;
        public static final int kCANrangeDeviceRightId = 42;

        public static final double kCANRangeStowElevatorThresholdMeters = 0.38;
        public static final double kCANRangeStowThresholdMeters = 0.43;

        public static final double kClawRefreshRateHz = 250;
        public static final double kClimberRefreshRateHz = 250;
        public static final double kIndexerRefreshRateHz = 100;
        public static final double kElevatorRefreshRateHz = 100;

        public static final double kCoralStageBannerDebounceTime =
                currentMode == Mode.REAL ? 0.03 : 0.0;
        public static final double kFirstIndexerBannerDebounceTime = 0.03;
        public static final double kSecondIndexerBannerDebounceTime = 0.03;
        public static final double kClimberHallEffectDebounceTime = 0.01;
        public static final double kClimberLatchDebounceTime = 0.1;
    }

    // Controls
    public static final boolean kForceDriveGamepad = true;
    public static final int kGamepadAdditionalControllerPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kDriveJoystickThreshold = 0.03;

    // April Tag Layout
    public static final AprilTagFieldLayout kAprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
            new AprilTagFieldLayout(
                    kAprilTagLayout.getTags().stream()
                            .filter(
                                    tag ->
                                            Arrays.stream(kAllowedTagIDs)
                                                    .anyMatch(element -> element == tag.ID))
                            .toList(),
                    kAprilTagLayout.getFieldLength(),
                    kAprilTagLayout.getFieldWidth());

    public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth();
    public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength();

    public static final double kReefRadius = 0.9604; // m

    // Limelight constants
    public static final class VisionConstants {

        // Large variance used to downweight unreliable vision measurements
        public static final double kLargeVariance = 1e6;

        // Standard deviation constants
        public static final int kMegatag1XStdDevIndex = 0;
        public static final int kMegatag1YStdDevIndex = 1;
        public static final int kMegatag1YawStdDevIndex = 5;

        // Standard deviation array indices for Megatag2
        public static final int kMegatag2XStdDevIndex = 6;
        public static final int kMegatag2YStdDevIndex = 7;
        public static final int kMegatag2YawStdDevIndex = 11;

        // Validation constants
        public static final int kExpectedStdDevArrayLength = 12;

        public static final int kMinFiducialCount = 1;

        // Camera A (Left-side Camera)
        public static final double kCameraAPitchDegrees = 20.0;
        public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
        public static final double kCameraAHeightOffGroundMeters = Units.inchesToMeters(8.3787);
        public static final String kLimelightATableName = "limelight-left";
        public static final double kRobotToCameraAForward = Units.inchesToMeters(7.8757);
        public static final double kRobotToCameraASide = Units.inchesToMeters(-11.9269);
        public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(0.0);
        public static final Transform2d kRobotToCameraA =
                new Transform2d(
                        new Translation2d(kRobotToCameraAForward, kRobotToCameraASide),
                        kCameraAYawOffset);

        // Camera B (Right-side camera)
        public static final double kCameraBPitchDegrees = 20.0;
        public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
        public static final double kCameraBHeightOffGroundMeters = Units.inchesToMeters(8.3787);
        public static final String kLimelightBTableName = "limelight-right";
        public static final double kRobotToCameraBForward = Units.inchesToMeters(7.8757);
        public static final double kRobotToCameraBSide = Units.inchesToMeters(11.9269);
        public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(0.0);
        public static final Transform2d kRobotToCameraB =
                new Transform2d(
                        new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide),
                        kCameraBYawOffset);

        // Vision processing constants
        public static final double kDefaultAmbiguityThreshold = 0.19;
        public static final double kDefaultYawDiffThreshold = 5.0;
        public static final double kTagAreaThresholdForYawCheck = 2.0;
        public static final double kTagMinAreaForSingleTagMegatag = 1.0;
        public static final double kDefaultZThreshold = 0.2;
        public static final double kDefaultNormThreshold = 1.0;
        public static final double kMinAmbiguityToFlip = 0.08;

        public static final double kCameraHorizontalFOVDegrees = 81.0;
        public static final double kCameraVerticalFOVDegrees = 55.0;
        public static final int kCameraImageWidth = 1280;
        public static final int kCameraImageHeight = 800;

        public static final double kScoringConfidenceThreshold = 0.7;

        // NetworkTables constants
        public static final String kBoundingBoxTableName = "BoundingBoxes";
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.74;
        public static final double kMaxAngularSpeedRadiansPerSecond = 6.5;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 31.538;

        public static final double kPXYController = 5.0;
        public static final double kPLTEController = 3.0;
        public static final double kPCTEController = 6.0;
        public static final double kPThetaController = 5.0;

        public static final double kTranslationKa = 0.0;
        public static final double kMaxEndPathVelocity = 2.0; // m/s

        public static final double kTriggerTimeBeforeEnd = 0.8; // seconds

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class LEDConstants {
        public static final CANDeviceId kCANdleId =
                new CANDeviceId(18, kCanBusDrivebaseClimberCanivore);
        public static final int kNonCandleLEDCount = 10;
        public static final int kCandleLEDCount = 8;
        public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;
        public static final double kLowBatteryThresholdVolts = 12.3;
    }

    public static final class ClawConstants {
        public static final double kCoralRollerDutyCycleRollback = -0.25;
        public static final double kCoralRollerDutyCycleScore = 0.75;
        public static final double kCoralRollerL1PrescoreRotations = -0.4;
        public static final double kCoralRollerL1ShallowRPS = -3.0;
        public static final double kCoralRollerL1DeepRPS = -5.0;
        public static final double kCoralRollerDutyCycleExhaust = 1.0;
        public static final double kCoralRollerVoltageHandoff = -10.0;

        public static final double kAlgaeRollerTorqueCurrentIntake = 70.0;
        public static final double kAlgaeRollerTorqueCurrentHold = 30.0;
        public static final double kAlgaeRollerTorqueCurrentScore = -50.0;
        public static final double kAlgaeRollerVoltageScoreBarge = -12.0;
        public static final double kAlgaeRollerTorqueCurrentExhaust = -50.0;
        public static final double kAlgaeRollerThresholdVelocityRPS = 6.0;
        public static final double kAlgaeRollerThresholdVoltage = 0.1;
        public static final double kAlgaeRollerDebounceTime = 0.25;
        public static final double kAlgaeRollerAutoDebounceTime = 0.1;
        public static final double kAlgaeRollerRegrabTimeSeconds = 4;
    }

    public static VoltageConfigs kDefaultClawVoltageConfigs = new VoltageConfigs();
    public static VoltageConfigs kL3_L2ClawVoltageConfigs = new VoltageConfigs();

    static {
        kDefaultClawVoltageConfigs.PeakForwardVoltage = 16;
        kDefaultClawVoltageConfigs.PeakReverseVoltage = -16;

        kL3_L2ClawVoltageConfigs.PeakForwardVoltage = 6;
        kL3_L2ClawVoltageConfigs.PeakReverseVoltage = -16;
    }

    public static ServoMotorSubsystemConfig kClawConfig = new ServoMotorSubsystemConfig();

    static {
        kClawConfig.name = "Claw";
        kClawConfig.talonCANID = new CANDeviceId(27, kCanBusSuperstructureCanivore);
        kClawConfig.momentOfInertia = 0.00023357;
        kClawConfig.unitToRotorRatio = ((12.0 / 48.0) * (15.0 / 36.0));

        // Used for auto score
        kClawConfig.fxConfig.Slot0.kP = 10.0;

        // Used for L1 RPS
        kClawConfig.fxConfig.Slot1.kP = 0.25;
        kClawConfig.fxConfig.Slot1.kS = 0.3;
        kClawConfig.fxConfig.Slot1.kV = 0.09;

        kClawConfig.fxConfig.HardwareLimitSwitch.ReverseLimitSource =
                ReverseLimitSourceValue.RemoteCANdiS1;
        kClawConfig.fxConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID =
                SensorConstants.kClawCoralCandiID;
        kClawConfig.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        kClawConfig.fxConfig.HardwareLimitSwitch.withReverseLimitAutosetPositionEnable(true);
        kClawConfig.fxConfig.HardwareLimitSwitch.ReverseLimitType =
                ReverseLimitTypeValue.NormallyOpen;
        kClawConfig.fxConfig.HardwareLimitSwitch.withReverseLimitAutosetPositionValue(-100.0);

        kClawConfig.fxConfig.Voltage = kDefaultClawVoltageConfigs;

        kClawConfig.fxConfig.HardwareLimitSwitch.ForwardLimitSource =
                ForwardLimitSourceValue.RemoteCANdiS2;
        kClawConfig.fxConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID =
                SensorConstants.kClawCoralCandiID;
        kClawConfig.fxConfig.HardwareLimitSwitch.withForwardLimitAutosetPositionEnable(false);
        kClawConfig.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        kClawConfig.fxConfig.HardwareLimitSwitch.ForwardLimitType =
                ForwardLimitTypeValue.NormallyOpen;

        kClawConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kClawConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kClawConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    }

    public static final class ClimberConstants {
        public static final double kClimberPivotClimbPositionRadians = Units.degreesToRadians(0);
        public static final double kClimberPivotDeployPositionRadians = Units.degreesToRadians(99);
        public static final double kClimberPivotReleaseSlackPositionRadians =
                Units.degreesToRadians(97);
        public static final double kClimberPivotToleranceRadians = 0.05;
        public static final double kClimberPivotDeployRotations = 46.2;
        public static final double kClimberRollerDutyCycleLatch = 1.0;
        public static final double kClimberPivotVoltageDeploy = 12;
        public static final double kClimberPivotVoltageRemoveSlack = 6;
        public static final double kClimberPivotVoltageStow = 12;
        public static final double kClimberPivotDutyCycleUntriggerHallEffect = 0.5;
        public static final double kClimberPivotRemoveSlackSeconds = 0.25;
        public static final double kClimberPivotCancoderOffset =
                kIsPracticeBot ? 0.300537 : -0.098389;
        public static final double kClimberPivotGearing =
                (11.0 / 54.0) * (14.0 / 58.0) * (0.27 / 2.74);
    }

    public static final ServoMotorSubsystemWithCanCoderConfig kClimberPivotConfig =
            new ServoMotorSubsystemWithCanCoderConfig();

    static {
        kClimberPivotConfig.name = "Climber_Pivot";
        kClimberPivotConfig.talonCANID = new CANDeviceId(14, kCanBusDrivebaseClimberCanivore);
        kClimberPivotConfig.momentOfInertia = 0.02330333;
        kClimberPivotConfig.fxConfig.Slot0.kP = 0.0;
        kClimberPivotConfig.fxConfig.Slot0.kD = 0.0;
        kClimberPivotConfig.unitToRotorRatio = Units.rotationsToRadians(1);

        kClimberPivotConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kClimberPivotConfig.canCoderConfig.CANID =
                new CANDeviceId(19, kCanBusDrivebaseClimberCanivore);
        kClimberPivotConfig.canCoderConfig.config.MagnetSensor.MagnetOffset =
                ClimberConstants.kClimberPivotCancoderOffset;
        kClimberPivotConfig.cancoderToUnitsRatio = Units.rotationsToRadians(1);

        kClimberPivotConfig.fxConfig.Feedback.FeedbackRemoteSensorID =
                kClimberPivotConfig.canCoderConfig.CANID.getDeviceNumber();
        kClimberPivotConfig.fxConfig.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.RemoteCANcoder;
        kClimberPivotConfig.fxConfig.Feedback.SensorToMechanismRatio = 1 / 1.0;

        kClimberPivotConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        kClimberPivotConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        kClimberPivotConfig.ratioForSim = ClimberConstants.kClimberPivotGearing;
        kClimberPivotConfig.cancoderUnitsForSim = 1.0;
    }

    public static final ServoMotorSubsystemConfig kClimberRollerConfig =
            new ServoMotorSubsystemConfig();

    static {
        kClimberRollerConfig.name = "Climber_Roller";
        kClimberRollerConfig.talonCANID = new CANDeviceId(15, kCanBusDrivebaseClimberCanivore);
        kClimberRollerConfig.momentOfInertia = 0.00042474;
        kClimberRollerConfig.unitToRotorRatio = ((10.0 / 30.0) * (15.0 / 30.0));

        kClimberRollerConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kClimberRollerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kClimberRollerConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kClimberRollerConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    }

    public static final class ElevatorConstants {
        public static final double kElevatorL1HeightMeters = 0.07;
        public static final double kElevatorL2HeightMeters = 0.13;
        public static final double kElevatorL2AutoAlignContinueHeightMeters = 0.0;
        public static final double kElevatorL2NonDescoreHeightMeters = 0.0;
        public static final double kElevatorL3HeightMeters = 0.52;
        public static final double kElevatorL3AutoAlignContinueHeightMeters = 0.0;
        public static final double kElevatorL3NonDescoreHeightMeters =
                kElevatorL3HeightMeters - .075;
        public static final double kElevatorL4HeightMeters = 1.14;
        public static final double kElevatorL4ClearanceHeightMeters = 1.2;
        public static final double kElevatorL4IntermediateHeightMeters = 1.25;
        public static final double kElevatorL4NonDescoreHeightMeters =
                kElevatorL4IntermediateHeightMeters;
        public static final double kElevatorToleranceMeters = 0.05;

        public static final double kElevatorMaxRotations = 31.3;
        public static final double kElevatorToleranceRotations = 1.0;

        public static final double kElevatorGroundHeightMeters = 0.1;

        public static final double kElevatorHomePositionMeters = 0.0;

        public static final double kElevatorDrumRadius = 0.02866242038;
        public static final double kGearing = (11.0 / 50.0);
        public static final double kElevatorUnitToRotorRatio =
                kGearing * 2.0 * kElevatorDrumRadius * Math.PI;
        public static final double kClearElevatorCrossbeamHeightMeters =
                0.96; // Upper bound 0.96, lower bound 0.265, wrist 1.967 rad max for lower bound
        public static final double kElevatorL4AutoAlignContinueHeightMeters = 0.1;
        public static final double kElevatorIntakeCoralHeightMeters = 0;
        public static final double kElevatorStowCoralHeightMeters = 0;
        public static final double kElevatorStowAlgaeHeightMeters = 0.0;
        public static final double kElevatorGroundAlgaeIntakeHeightMeters = 0;
        public static final double kElevatorLollipopIntakeHeightMeters = 0.3;
        public static final double kElevatorReefAlgaeIntakeL2HeightMeters = 0.606;
        public static final double kElevatorReefSlap = 0.506;
        public static final double kElevatorReefAlgaeIntakeL3HeightMeters = 1.006;
        public static final double kElevatorReefAlgaeIntakeL3ManualHeightMeters = 0.5319;
        public static final double kElevatorBargeHeightMeters = 1.385;
        public static final double kElevatorReleaseAlgaeHeightMeters = 0.75;
        public static final double kElevatorClearBargeHeightMeters = 1.24;
        public static final double kElevatorProcessorHeightMeters = 0.15;
        public static final double kClearWheelsHeightMeters = 0.0;
        public static final double kClearLimelightHeightMeters = 0.0;
    }

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig
            kElevatorFollowerRightConfig =
                    new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    static {
        kElevatorFollowerRightConfig.config.name = "Elevator_Right";
        kElevatorFollowerRightConfig.config.talonCANID =
                new CANDeviceId(22, kCanBusSuperstructureCanivore);
        kElevatorFollowerRightConfig.config.momentOfInertia = 0.04;
        kElevatorFollowerRightConfig.config.unitToRotorRatio =
                ElevatorConstants.kElevatorUnitToRotorRatio;
        kElevatorFollowerRightConfig.inverted = true;

        kElevatorFollowerRightConfig.config.fxConfig.MotorOutput.NeutralMode =
                NeutralModeValue.Brake;
        kElevatorFollowerRightConfig.config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kElevatorFollowerRightConfig.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        kElevatorFollowerRightConfig.config.fxConfig.HardwareLimitSwitch.ReverseLimitSource =
                ReverseLimitSourceValue.RemoteCANdiS1;
        kElevatorFollowerRightConfig
                        .config
                        .fxConfig
                        .HardwareLimitSwitch
                        .ReverseLimitRemoteSensorID =
                Constants.SensorConstants.kElevatorCandiID;
        kElevatorFollowerRightConfig.config.fxConfig.HardwareLimitSwitch.ReverseLimitType =
                ReverseLimitTypeValue.NormallyOpen;
    }

    public static MotionMagicConfigs kDefaultElevatorConfig = new MotionMagicConfigs();
    public static MotionMagicConfigs kLowElevatorConfig = new MotionMagicConfigs();

    static {
        kDefaultElevatorConfig.MotionMagicCruiseVelocity = 90.0;
        kDefaultElevatorConfig.MotionMagicAcceleration = 1000.0;
        kDefaultElevatorConfig.MotionMagicJerk = 3600.0;

        kLowElevatorConfig.MotionMagicCruiseVelocity = 90.0;
        kLowElevatorConfig.MotionMagicAcceleration = 1000.0;
        kLowElevatorConfig.MotionMagicJerk = 7200.0;
    }

    public static final ServoMotorSubsystemWithFollowersConfig kElevatorConfig =
            new ServoMotorSubsystemWithFollowersConfig();

    static {
        kElevatorConfig.name = "Elevator";
        kElevatorConfig.talonCANID = new CANDeviceId(21, kCanBusSuperstructureCanivore);
        kElevatorConfig.unitToRotorRatio = ElevatorConstants.kElevatorUnitToRotorRatio;
        kElevatorConfig.fxConfig.Slot0.kG = 0.325;
        kElevatorConfig.fxConfig.Slot0.kS = 0.075;
        kElevatorConfig.fxConfig.Slot0.kV = 0.1333;
        kElevatorConfig.fxConfig.Slot0.kP = 5.0;
        kElevatorConfig.fxConfig.Slot0.kA = 0.005;

        kElevatorConfig.fxConfig.MotionMagic = kDefaultElevatorConfig;

        kElevatorConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kElevatorConfig.kMinPositionUnits = 0.0;
        kElevatorConfig.kMaxPositionUnits = 1.39;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kElevatorConfig.kMaxPositionUnits / kElevatorConfig.unitToRotorRatio;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kElevatorConfig.kMinPositionUnits / kElevatorConfig.unitToRotorRatio;

        kElevatorConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kElevatorConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kElevatorConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        kElevatorConfig.fxConfig.HardwareLimitSwitch.ReverseLimitSource =
                ReverseLimitSourceValue.RemoteCANdiS1;
        kElevatorConfig.fxConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID =
                Constants.SensorConstants.kElevatorCandiID;
        kElevatorConfig.fxConfig.HardwareLimitSwitch.ReverseLimitType =
                ReverseLimitTypeValue.NormallyOpen;

        kElevatorConfig.followers =
                new ServoMotorSubsystemWithFollowersConfig.FollowerConfig[] {
                    kElevatorFollowerRightConfig
                };
    }

    public static final SimElevator.SimElevatorConfig kSimElevatorConfig =
            new SimElevator.SimElevatorConfig();

    static {
        kSimElevatorConfig.carriageMass = 1.97312681;
        kSimElevatorConfig.drumRadius = ElevatorConstants.kElevatorDrumRadius;
        kSimElevatorConfig.gearing = ElevatorConstants.kGearing;
        kSimElevatorConfig.meterToRotorRatio = ElevatorConstants.kElevatorUnitToRotorRatio;
    }

    public static final class WristConstants {
        public static final double kWristZStartingOffset = 0.69; // in m
        public static final double kWristLength = Units.inchesToMeters(18.125);
        public static final double kWristStageBargePositionRadians =
                Units.degreesToRadians(132 - 90);
        public static final double kWristScoreBargePositionRadians = Units.degreesToRadians(113);
        public static final double kWristProcessorPositionRadians = Units.degreesToRadians(24 - 90);
        public static final double kWristL1PositionRadians = Units.degreesToRadians(-93);
        public static final double kWristL2PositionRadians = Units.degreesToRadians(86 - 90);
        public static final double kWristL2SlapRadians = Units.degreesToRadians(20);
        public static final double kWristL2NonDescorePositionRadians = Units.degreesToRadians(55);
        public static final double kWristL3PositionRadians = Units.degreesToRadians(86 - 90);
        public static final double kClearL3DescoreForAlgaePositionRadians =
                Units.degreesToRadians(0);
        public static final double kWristL3NonDescorePositionRadians = Units.degreesToRadians(55);
        public static final double kWristL4PositionRadians = Units.degreesToRadians(88 - 90);
        public static final double kWristL4IntermediatePositionRadians =
                Units.degreesToRadians(85 - 90);
        public static final double kWristDescoreClearancePositionRadians =
                Units.degreesToRadians(3);
        public static final double kWristL4NonDescorePositionRadians = Units.degreesToRadians(55);
        public static final double kWristToleranceRadians = 0.05;
        public static final double kClearElevatorCrossbeamMinPositionRadians =
                Units.degreesToRadians(0 - 90);
        public static final double kClearElevatorCrossbeamMaxPositionRadians =
                Units.degreesToRadians(10 - 90);
        public static final double kWristIntakeCoralPositionRadians =
                Units.degreesToRadians(-13 - 90);
        public static final double kWristStowCoralPositionRadians =
                Units.degreesToRadians(-13 - 90);
        public static final double kWristStowAlgaePositionRadians = Units.degreesToRadians(55);
        public static final double kWristGroundAlgaeIntakePositionRadians =
                Units.degreesToRadians(19 - 90);
        public static final double kWristLollipopIntakePositionRadians =
                Units.degreesToRadians(19 - 90);
        public static final double kWristReefAlgaeIntakeL2PositionRadians =
                Units.degreesToRadians(18.7 - 90);
        public static final double kWristReefAlgaeIntakeL3PositionRadians =
                Units.degreesToRadians(18.7 - 90);
        public static final double kWristReefAlgaeIntakeL3ManualPositionRadians =
                Units.degreesToRadians(86 - 90);
        public static final double kWristClearBargePositionRadians =
                Units.degreesToRadians(180 - 90);
        public static final double kClearWheelsPositionRadians = Units.degreesToRadians(-87);
        public static final double kClearLimelightPositionRadians =
                Units.degreesToRadians(132 - 90);
        public static final double kWristRatio = (10.0 / 48.0) * (14.0 / 32.0) * (10.0 / 50.0);
        public static final double kCancoderRatio = 1.0 / 1.0;
        public static final double kWristCancoderOffset =
                (kIsPracticeBot ? -0.249512 : 0.409668) - 0.25;
    }

    public static final ServoMotorSubsystemWithCanCoderConfig kWristConfig =
            new ServoMotorSubsystemWithCanCoderConfig();

    public static MotionMagicConfigs kDefaultWristConfig = new MotionMagicConfigs();
    public static MotionMagicConfigs kHasAlgaeWristConfig = new MotionMagicConfigs();

    static {
        kDefaultWristConfig.MotionMagicCruiseVelocity = 80.0 * WristConstants.kWristRatio;
        kDefaultWristConfig.MotionMagicAcceleration = 1000.0 * WristConstants.kWristRatio;
        kDefaultWristConfig.MotionMagicJerk = 4800.0 * WristConstants.kWristRatio;

        kHasAlgaeWristConfig.MotionMagicCruiseVelocity = 80.0 * WristConstants.kWristRatio;
        kHasAlgaeWristConfig.MotionMagicAcceleration = 600.0 * WristConstants.kWristRatio;
        kHasAlgaeWristConfig.MotionMagicJerk = 4000.0 * WristConstants.kWristRatio;
    }

    static {
        kWristConfig.name = "Wrist";
        kWristConfig.talonCANID = new CANDeviceId(26, kCanBusSuperstructureCanivore);
        kWristConfig.fxConfig.Slot0.kP = 100.0; // 10.0 / WristConstants.kWristRatio;
        kWristConfig.fxConfig.Slot0.kS = 0.3;
        kWristConfig.fxConfig.Slot0.kI = 0;
        kWristConfig.fxConfig.Slot0.kD = 0.0;
        kWristConfig.fxConfig.Slot0.kG = 0.37;
        kWristConfig.fxConfig.Slot0.kV = 0.075 / WristConstants.kWristRatio;
        kWristConfig.fxConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        kWristConfig.fxConfig.Slot1.kP = 300.0;
        kWristConfig.fxConfig.Slot1.kS = kWristConfig.fxConfig.Slot0.kS;
        kWristConfig.fxConfig.Slot1.kI = 0.0;
        kWristConfig.fxConfig.Slot1.kD = kWristConfig.fxConfig.Slot0.kD;
        kWristConfig.fxConfig.Slot1.kG = kWristConfig.fxConfig.Slot0.kG;
        kWristConfig.fxConfig.Slot1.kV = kWristConfig.fxConfig.Slot0.kV;
        kWristConfig.fxConfig.Slot1.GravityType = kWristConfig.fxConfig.Slot0.GravityType;

        kWristConfig.momentOfInertia = 0.01;
        kWristConfig.unitToRotorRatio = Units.rotationsToRadians(1);

        kWristConfig.kMaxPositionUnits = Math.toRadians(140.0);
        kWristConfig.kMinPositionUnits = Units.degreesToRadians(-104);
        kWristConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kWristConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(kWristConfig.kMaxPositionUnits);
        kWristConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kWristConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(kWristConfig.kMinPositionUnits);

        kWristConfig.fxConfig.MotionMagic = kDefaultWristConfig;

        kWristConfig.canCoderConfig.CANID = new CANDeviceId(29, kCanBusSuperstructureCanivore);
        kWristConfig.canCoderConfig.config.MagnetSensor.MagnetOffset =
                WristConstants.kWristCancoderOffset;
        kWristConfig.canCoderConfig.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        kWristConfig.cancoderToUnitsRatio = Units.rotationsToRadians(WristConstants.kCancoderRatio);

        kWristConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kWristConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kWristConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

        kWristConfig.fxConfig.Feedback.FeedbackRemoteSensorID =
                kWristConfig.canCoderConfig.CANID.getDeviceNumber();
        kWristConfig.fxConfig.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.FusedCANcoder;
        kWristConfig.fxConfig.Feedback.SensorToMechanismRatio = 1.0 / WristConstants.kCancoderRatio;
        kWristConfig.fxConfig.Feedback.RotorToSensorRatio =
                (1 / WristConstants.kWristRatio) * WristConstants.kCancoderRatio;
        kWristConfig.isFusedCancoder = true;
        kWristConfig.ratioForSim = WristConstants.kWristRatio;
        kWristConfig.cancoderUnitsForSim = WristConstants.kCancoderRatio;
    }

    public static final class IndexerConstants {
        public static final double kIndexerDutyCycle = 1.0;
        public static final double kIndexerDutyCycleExhaust = -1.0;
        public static final double kIndexerRadius = 0.0635; // in m
    }

    public static final ServoMotorSubsystemConfig kIndexerConfig = new ServoMotorSubsystemConfig();

    static {
        kIndexerConfig.name = "Indexer";
        kIndexerConfig.talonCANID = new CANDeviceId(25, kCanBusSuperstructureCanivore);
        kIndexerConfig.momentOfInertia = 0.000552;
        kIndexerConfig.unitToRotorRatio = (12.0 / 48.0);

        kIndexerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kIndexerConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kIndexerConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 80.0;

        kIndexerConfig.fxConfig.HardwareLimitSwitch.ForwardLimitSource =
                ForwardLimitSourceValue.RemoteCANdiS2;
        kIndexerConfig.fxConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID =
                SensorConstants.kIndexerCandiID;
        kIndexerConfig.fxConfig.HardwareLimitSwitch.ForwardLimitType =
                ForwardLimitTypeValue.NormallyOpen;
        kIndexerConfig.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    }

    public static final class IntakeConstants {
        public static final double kIntakeDutyCycleIntake = 1.0;
        public static final double kIntakeDutyCycleExhaust = -0.75;

        public static final double kIntakePivotStowPositionRadians = Units.degreesToRadians(-80);
        public static final double kIntakePivotStowForClimbPositionRadians =
                Units.degreesToRadians(-90);
        public static final double kIntakePivotDeployPositionRadians = 0.0;
        public static final double kIntakePivotCoralPushedPositionRadians =
                Units.degreesToRadians(-0.5);
        public static final double kIntakePivotLollipopDeployPositionRadians =
                Units.degreesToRadians(30);
        public static final double kIntakePivotToleranceRadians = 0.05;
        public static final double kIntakeRollerRadius = 0.0269875; // in m

        public static final double kIntakePivotCancoderOffset =
                kIsPracticeBot ? 0.411133 : -0.086914;
    }

    public static final ServoMotorSubsystemConfig kIntakeRollerConfig =
            new ServoMotorSubsystemConfig();

    static {
        kIntakeRollerConfig.name = "Intake_Roller";
        kIntakeRollerConfig.talonCANID = new CANDeviceId(24, kCanBusSuperstructureCanivore);
        kIntakeRollerConfig.momentOfInertia = 0.00132536;
        kIntakeRollerConfig.unitToRotorRatio = (12.0 / 24.0);

        kIntakeRollerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kIntakeRollerConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kIntakeRollerConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    }

    public static final ServoMotorSubsystemWithCanCoderConfig kIntakePivotConfig =
            new ServoMotorSubsystemWithCanCoderConfig();

    static {
        kIntakePivotConfig.name = "Intake_Pivot";
        kIntakePivotConfig.talonCANID = new CANDeviceId(23, kCanBusSuperstructureCanivore);
        kIntakePivotConfig.momentOfInertia = 0.01;
        kIntakePivotConfig.fxConfig.Slot0.kP = 2.0;
        kIntakePivotConfig.fxConfig.Slot0.kD = 0.3;
        kIntakePivotConfig.fxConfig.Slot0.kV = 0.15;
        kIntakePivotConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0;
        kIntakePivotConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0;
        kIntakePivotConfig.unitToRotorRatio =
                Units.rotationsToRadians((10.0 / 36.0) * (14.0 / 42.0) * (14.0 / 56.0));

        kIntakePivotConfig.canCoderConfig.CANID =
                new CANDeviceId(30, kCanBusSuperstructureCanivore);
        kIntakePivotConfig.canCoderConfig.config.MagnetSensor.MagnetOffset =
                IntakeConstants.kIntakePivotCancoderOffset;
        kIntakePivotConfig.cancoderToUnitsRatio = Units.rotationsToRadians(1);

        kIntakePivotConfig.kMaxPositionUnits = 0.0;
        kIntakePivotConfig.kMinPositionUnits = Units.degreesToRadians(-115.6);
        kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kIntakePivotConfig.kMaxPositionUnits / kIntakePivotConfig.unitToRotorRatio;
        kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kIntakePivotConfig.kMinPositionUnits / kIntakePivotConfig.unitToRotorRatio;

        kIntakePivotConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kIntakePivotConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kIntakePivotConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kIntakePivotConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    }

    public static final class SimConstants {
        // Game piece simulation constants
        public static final double kGamePieceOffsetX = 0.2;
        public static final double kGamePieceOffsetZ = 0.35;
        public static final double kGamePieceInitialPitch = Math.PI / 3;
        public static final double kGamePieceInitialYaw = -Math.PI / 4;

        // Center pose offsets
        public static final double kCenterPoseOffsetX = 0.2;
        public static final double kCenterPoseZ = 0.7;

        // Claw height offset
        public static final double kClawHeightOffset = 0.690;

        // Invalid position for game piece (when not held)
        public static final Pose3d kInvalidGamePiecePose =
                new Pose3d(1000, 1000, 1000, new Rotation3d(0, 0, 0));

        // Voltage thresholds
        public static final double kIntakeRPSThreshold = 1.0;
        public static final double kAlgaeIntakeRPSThreshold = 5.0;

        // Scoring heights for different reef levels
        public static final double kProjectileHeightL1 = ElevatorConstants.kElevatorL1HeightMeters;
        public static final double kProjectileHeightL2 = 1.08;
        public static final double kProjectileHeightL3 = 1.37;
        public static final double kProjectileHeightL4 = 2.1;

        // Projectile constants
        public static final double kProjectileOffsetXL1 = 0.35;
        public static final double kProjectileOffsetXL2 = 0.35;
        public static final double kProjectileOffsetXL3 = 0.35;
        public static final double kProjectileOffsetXL4 = 0.46;

        public static final double kProjectileAngleL1 = -35;
        public static final double kProjectileAngleL2 = -35;
        public static final double kProjectileAngleL3 = -35;
        public static final double kProjectileAngleL4 = -90.0;
        public static final double kCoralProjectileSpeed = 3.0;

        public static final double kRequiredExhaustTime = 0.5;

        // Game piece dimensions (Coral)
        public static final double kCoralLengthMeters =
                Units.inchesToMeters(11.875); // Length of coral
        public static final double kCoralDiameterMeters =
                Units.inchesToMeters(4.5); // Outer diameter of coral
        public static final double kBranchContactThresholdMeters =
                0.2; // Contact threshold for any part of coral

        // Scoring thresholds
        public static final double kHighBranchHeightThreshold = 1.5; // meters
        public static final double kHighBranchContactThreshold = 0.03; // 3cm
        public static final double kLowBranchContactThreshold = 0.05; // 5cm
        public static final double kHighBranchVerticalTolerance = kCoralLengthMeters / 3.0;
        public static final double kLowBranchVerticalTolerance = kCoralLengthMeters / 2.0;
        public static final double kMaxScoringAngle = Math.PI / 6; // 30 degrees

        public static final double kIntakeLengthMeters = 0.7;
        public static final double kIntakeWidthMeters = 0.2;

        // Algae simulation constants

        public static final double kAlgaeProjectileSpeed = 4.0;
        public static final double kAlgaeProjectileOffsetX = 0.35;
        public static final double kProcessorScoringTolerance = 0.1; // meters

        // HP Intake Constants
        public static final double kHPIntakeDistanceThreshold = 1;
        public static final double kHPIntakeAngleThreshold = Math.PI / 6;

        // Funnel Simulation Constants
        public static final double kFunnelStartX = -0.7;
        public static final double kFunnelEndX = 0;
        public static final double kFunnelStartHeight = 1.4;
        public static final double kFunnelEndHeight = 0.25;
        public static final double kFunnelSlopeAngle = Math.toRadians(50);
        public static final double kFunnelFrictionCoeff = 0.15;
        public static final double kFunnelInitialVelocity = 0.1;
    }

    public static final Pose2d kFeederRightPose =
            new Pose2d(
                    1.7196709632873535,
                    0.6049244999885559,
                    Rotation2d.fromDegrees(
                            60)); // new Pose2d(1.5196709632873535, 0.6049244999885559,
    // Rotation2d.fromDegrees(54));
    public static final Pose2d kFeederLeftPose =
            new Pose2d(
                    1.7196709632873535,
                    7.364640235900879,
                    Rotation2d.fromDegrees(
                            -60)); // new Pose2d(1.5196709632873535, 7.364640235900879,
    // Rotation2d.fromDegrees(-54));
    public static final Pose2d kBargePose = new Pose2d(8.0, 6.0, Rotation2d.k180deg);

    public static final Pose2d[] kBranchPoses = getBranchPoses();
    public static final Pose2d[] kAlgaePoses = getReefCenterPoses();
    public static final double kAutoAlignReefBuffer = 1.0; // m
    public static final double kAutoAlignReefBufferAuto = 1.5; // m
    public static final double kAutoAlignReefUnbuffer = 1.0; // m
    public static final double kAutoAlignFeederBufferAuto = 0.2; // m
    public static final double kAutoAlignFeederGroundBufferAuto = 0.2; // m
    public static final double kAutoAlignFeederGroundBufferFirstPathAuto = 0.5; // m
    public static final double kAutoAlignFeederBufferTeleop = 0.1; // m
    public static final double kAutoAlignReefBackoffDistance = -0.3; // m
    public static final double kAutoAlignAlgaeReefBackoffDistance = -0.65; // m
    public static final double kMidlineBuffer = 1.0;
    public static final double kRobotSafetyPadding = 0.1;
    public static final double kReefVisibilityBuffer =
            kRobotWidth * Math.sqrt(2) / 2.0 + kRobotSafetyPadding; // m

    public static Pose2d[] getReefCenterPoses() {
        Pose2d[] reefPoses = new Pose2d[Reef.centerFaces.length];

        for (int i = 0; i < reefPoses.length; i++) {
            Pose2d reefPose = Reef.centerFaces[i];

            Transform2d offsetTransform =
                    new Transform2d(
                            new Translation2d(
                                    Constants.kRobotWidth / 2.0 + Constants.kAutoAlignReefBuffer,
                                    0),
                            Rotation2d.k180deg);

            reefPoses[i] = reefPose.plus(offsetTransform);
        }

        return reefPoses;
    }

    public static Pose2d[] getBranchPoses() {
        Pose2d[] branchPoses = new Pose2d[Reef.branchPositions.size()];

        for (int i = 0; i < branchPoses.length; i++) {
            Pose3d branchPose3d = Reef.branchPositions.get(i).get(ReefHeight.L3);

            Translation2d branchTranslation2d =
                    new Translation2d(branchPose3d.getX(), branchPose3d.getY());

            Rotation2d branchRotation2d = new Rotation2d(branchPose3d.getRotation().getZ());

            branchPoses[i] = new Pose2d(branchTranslation2d, branchRotation2d);

            Transform2d offsetTransform =
                    new Transform2d(
                            new Translation2d(
                                    Constants.kRobotWidth / 2.0 + Constants.kAutoAlignReefBuffer,
                                    0),
                            Rotation2d.k180deg);

            branchPoses[i] = branchPoses[i].plus(offsetTransform);
        }

        return branchPoses;
    }

    /**
     * Check if this system has a certain mac address in any network device.
     *
     * @param mac_address Mac address to check.
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String mac_address) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder device_mac_sb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        device_mac_sb.append(
                                String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String device_mac = device_mac_sb.toString();
                    System.out.println(
                            "hasMacAddress: NIS "
                                    + nis.getDisplayName()
                                    + " device_mac: "
                                    + device_mac);
                    if (mac_address.equals(device_mac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }
}
