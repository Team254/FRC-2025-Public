package com.team254.frc2025.simulation;

import com.team254.frc2025.Constants;
import com.team254.frc2025.Constants.ElevatorConstants;
import com.team254.frc2025.Constants.SimConstants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.controlboard.ModalControls;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.pathplanner.util.FlippingUtil;
import com.team254.lib.pathplanner.util.GeometryUtil;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.FieldConstants;
import com.team254.lib.util.ReefscapeUtil;
import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.Optional;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class SimulatedRobotState {
    TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth =
            TimeInterpolatableBuffer.createBuffer(RobotState.LOOKBACK_TIME);

    // If set, always assume picking up game piece when intake runs.
    private static final boolean kForceCoralAlways = true;
    private static final boolean kForceAlgaeAlways = true;
    private static final boolean kForceCoralJam = false;

    private enum ClimberState {
        DEPLOYING,
        IN_SLACK,
        STOWING
    }

    private enum CoralState {
        NO_CORAL,
        CORAL_IN_FUNNEL,
        CORAL_IN_INTAKE,
        CORAL_IN_INDEXER,
        CORAL_IN_CLAW
    }

    private enum AlgaeState {
        NO_ALGAE,
        ALGAE_IN_WRIST
    }

    private CoralState coralState = CoralState.NO_CORAL;
    private ClimberState climberState = ClimberState.DEPLOYING;
    private double climberUnspoolDistance = 0.0;

    private IntakeSimulation coralIntakeSimulation;
    private IntakeSimulation algaeIntakeSimulation;
    private SwerveDriveSimulation simDrive;
    private final RobotContainer container;
    private double contactStartTime = 0;
    private FieldConstants.ReefHeight currentContactHeight = null;
    private static final double REQUIRED_CONTACT_TIME = 0.01;
    private boolean gamePieceInClaw = false;
    private double lastTimestamp = 0.0;
    double intakeDistanceM = 0.3;
    double indexerDistanceM = 0.5;
    boolean readyForHandoff = false;

    // Linear distance from start that the coral is.
    private Optional<Double> coralLocation = Optional.empty();

    // Flag to track if we're currently in funnel mode
    private boolean inFunnelMode = false;

    private AlgaeState algaeState = AlgaeState.NO_ALGAE;
    private boolean algaeInRobot = false;

    // Add a timer for coral exhaust
    private double coralExhaustStartTime = 0.0;
    private boolean coralExhaustTimerActive = false;

    public SimulatedRobotState(RobotContainer container) {
        this.container = container;
    }

    // Do this after construction to avoid circular dependencies.
    public void init() {
        this.simDrive = this.container.getDriveSubsystem().getMapleSimDrive().mapleSimDrive;
        coralIntakeSimulation =
                IntakeSimulation.OverTheBumperIntake(
                        "Coral",
                        this.simDrive,
                        Units.Meters.of(SimConstants.kIntakeLengthMeters),
                        Units.Meters.of(SimConstants.kIntakeWidthMeters),
                        IntakeSimulation.IntakeSide.BACK,
                        1); // Capacity of 1 game piece
        algaeIntakeSimulation =
                IntakeSimulation.OverTheBumperIntake(
                        "Algae",
                        this.simDrive,
                        Units.Meters.of(SimConstants.kIntakeLengthMeters),
                        Units.Meters.of(SimConstants.kIntakeWidthMeters),
                        IntakeSimulation.IntakeSide.FRONT,
                        1); // Capacity of 1 game piece
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        fieldToRobotSimulatedTruth.addSample(RobotTime.getTimestampSeconds(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        var entry = fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
        if (entry == null) {
            return null;
        }
        return entry.getValue();
    }

    public FieldConstants.ReefHeight calculateBranchContact(Pose3d gamePiecePose) {
        double coralRadius = SimConstants.kCoralDiameterMeters / 2.0;
        boolean anyContact = false;
        FieldConstants.ReefHeight contactHeight = null;

        for (Map<FieldConstants.ReefHeight, Pose3d> branchMap :
                FieldConstants.Reef.branchPositions) {
            for (Map.Entry<FieldConstants.ReefHeight, Pose3d> entry : branchMap.entrySet()) {
                Pose3d branchPose = entry.getValue();
                if (calculateBranchCollision(gamePiecePose, branchPose, coralRadius)) {
                    anyContact = true;
                    contactHeight = entry.getKey();
                    break;
                }
            }
            if (anyContact) break;
        }

        return processContactDuration(anyContact, contactHeight);
    }

    private boolean calculateBranchCollision(
            Pose3d coralPose, Pose3d branchPose, double coralRadius) {
        var delta =
                new Translation3d(
                        coralPose.getX() - branchPose.getX(),
                        coralPose.getY() - branchPose.getY(),
                        coralPose.getZ() - branchPose.getZ());

        double horizontalDistance = Math.hypot(delta.getX(), delta.getY());

        // Calculate coral alignment
        double coralYaw = coralPose.getRotation().getZ();
        double angleToBranch =
                Math.acos(
                        (Math.cos(coralYaw) * delta.getX() + Math.sin(coralYaw) * delta.getY())
                                / horizontalDistance);

        if (Math.abs(angleToBranch) > SimConstants.kMaxScoringAngle) {
            return false;
        }

        boolean isHighBranch = branchPose.getZ() > SimConstants.kHighBranchHeightThreshold;
        double contactThreshold =
                isHighBranch
                        ? SimConstants.kHighBranchContactThreshold
                        : SimConstants.kLowBranchContactThreshold;

        if (Math.abs(horizontalDistance - coralRadius) > contactThreshold) {
            return false;
        }

        double verticalTolerance =
                isHighBranch
                        ? SimConstants.kHighBranchVerticalTolerance
                        : SimConstants.kLowBranchVerticalTolerance;

        if (Math.abs(delta.getZ()) > verticalTolerance) {
            return false;
        }

        return true;
    }

    private FieldConstants.ReefHeight processContactDuration(
            boolean anyContact, FieldConstants.ReefHeight contactHeight) {
        double currentTime = Timer.getFPGATimestamp();

        if (anyContact) {
            if (currentContactHeight == null) {
                contactStartTime = currentTime;
                currentContactHeight = contactHeight;
            } else if (currentContactHeight != contactHeight) {
                contactStartTime = currentTime;
                currentContactHeight = contactHeight;
            }
            if (currentTime - contactStartTime >= REQUIRED_CONTACT_TIME) {
                Logger.recordOutput(
                        "SimulatedRobotState/Collision/ContactTime",
                        currentTime - contactStartTime);
                return currentContactHeight;
            }
        } else {
            currentContactHeight = null;
        }
        return null;
    }

    public Pose3d calculateGamePiecePositionInClaw(Pose2d robotPose, Pose3d clawPose) {
        // Initial position calculation
        var gamePickupOffset =
                new Pose3d(
                        SimConstants.kGamePieceOffsetX,
                        0,
                        SimConstants.kGamePieceOffsetZ,
                        new Rotation3d(0, 0, SimConstants.kGamePieceInitialPitch));

        Pose3d gamePiecePose =
                new Pose3d(
                        robotPose.getX() + gamePickupOffset.getX(),
                        robotPose.getY() + gamePickupOffset.getY(),
                        gamePickupOffset.getZ(),
                        new Rotation3d(0, SimConstants.kGamePieceInitialYaw, 0));

        // Calculate center point for rotation
        Pose3d centerPose =
                new Pose3d(
                        robotPose.getX() + SimConstants.kCenterPoseOffsetX,
                        robotPose.getY(),
                        SimConstants.kCenterPoseZ,
                        new Rotation3d());

        // Apply claw-based rotation
        gamePiecePose = applyClawRotation(gamePiecePose, centerPose, clawPose);

        // Apply robot rotation
        return applyRobotRotation(gamePiecePose, robotPose);
    }

    private Pose3d applyClawRotation(Pose3d gamePiecePose, Pose3d centerPose, Pose3d clawPose) {
        double dx = gamePiecePose.getX() - centerPose.getX();
        double dy = gamePiecePose.getY() - centerPose.getY();
        double dz = gamePiecePose.getZ() - centerPose.getZ();
        double radius = Math.sqrt(dx * dx + dy * dy + dz * dz);

        double rotationAmount = Math.acos(clawPose.getRotation().getQuaternion().getW()) * 2.0;
        double originalAngle = Math.atan2(dz, dx);
        double newAngle = originalAngle + rotationAmount;

        return new Pose3d(
                centerPose.getX() + radius * Math.cos(newAngle),
                gamePiecePose.getY(),
                centerPose.getZ()
                        + radius * Math.sin(newAngle)
                        + clawPose.getZ()
                        - SimConstants.kClawHeightOffset,
                new Rotation3d(0, -(-Math.PI / 4 + newAngle), 0));
    }

    private Pose3d applyRobotRotation(Pose3d gamePiecePose, Pose2d robotPose) {
        double robotRotation = robotPose.getRotation().getRadians();

        double offsetX = gamePiecePose.getX() - robotPose.getX();
        double offsetY = gamePiecePose.getY() - robotPose.getY();

        double rotatedX = offsetX * Math.cos(robotRotation) - offsetY * Math.sin(robotRotation);
        double rotatedY = offsetX * Math.sin(robotRotation) + offsetY * Math.cos(robotRotation);

        return new Pose3d(
                robotPose.getX() + rotatedX,
                robotPose.getY() + rotatedY,
                gamePiecePose.getZ(),
                new Rotation3d(
                        gamePiecePose.getRotation().getX(),
                        gamePiecePose.getRotation().getY(),
                        robotRotation));
    }

    public void scoreCoralSim(FieldConstants.ReefHeight height) {
        Logger.recordOutput("Sim/LastDetectedContactHeight", height);
        double projectileHeight;
        double offsetX;
        double offsetAngle;
        switch (height) {
            case L1:
                projectileHeight = SimConstants.kProjectileHeightL1;
                offsetX = SimConstants.kProjectileOffsetXL1;
                offsetAngle = SimConstants.kProjectileAngleL1;
                break;
            case L2:
                projectileHeight = SimConstants.kProjectileHeightL2;
                offsetX = SimConstants.kProjectileOffsetXL2;
                offsetAngle = SimConstants.kProjectileAngleL2;
                break;
            case L3:
                projectileHeight = SimConstants.kProjectileHeightL3;
                offsetX = SimConstants.kProjectileOffsetXL3;
                offsetAngle = SimConstants.kProjectileAngleL3;
                break;
            case L4:
                projectileHeight = SimConstants.kProjectileHeightL4;
                offsetX = SimConstants.kProjectileOffsetXL4;
                offsetAngle = SimConstants.kProjectileAngleL4;
                break;
            default:
                projectileHeight = SimConstants.kProjectileHeightL1;
                offsetX = SimConstants.kProjectileOffsetXL4;
                offsetAngle = SimConstants.kProjectileAngleL4;
        }

        SimulatedArena.getInstance()
                .addGamePieceProjectile(
                        new ReefscapeCoralOnFly(
                                this.simDrive.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(offsetX, 0),
                                this.simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                this.simDrive.getSimulatedDriveTrainPose().getRotation(),
                                Units.Meters.of(projectileHeight),
                                Units.MetersPerSecond.of(SimConstants.kCoralProjectileSpeed),
                                Units.Degrees.of(offsetAngle)));
        gamePieceInClaw = false;

        // If we have scored a coral, we need to set the rotations for autoscore.
        container.getClaw().setCurrentPosition(-10.0);
    }

    protected void checkL1Coral() {
        // Must be positive
        if (container.getStateMachine().getCurrentState() == SuperstructureState.STAGE_CORAL_L1
                && container.getClaw().getCurrentVelocity() < -1.0) {
            gamePieceInClaw = false;
            double wristAngleRad = container.getWrist().getCurrentPosition();
            double projectileHeight =
                    container.getRobotState().getElevatorHeightMeters()
                            + Constants.WristConstants.kWristZStartingOffset
                            + Math.sin(wristAngleRad) * Constants.WristConstants.kWristLength
                            + 0.25;
            double shooterAngle =
                    new Rotation2d(wristAngleRad)
                            .rotateBy(Rotation2d.fromDegrees(125))
                            .getRadians();

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                    this.simDrive.getSimulatedDriveTrainPose().getTranslation(),
                                    new Translation2d(SimConstants.kProjectileOffsetXL1, 0),
                                    this.simDrive
                                            .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                    this.simDrive.getSimulatedDriveTrainPose().getRotation(),
                                    Units.Meters.of(projectileHeight),
                                    Units.MetersPerSecond.of(SimConstants.kCoralProjectileSpeed),
                                    Units.Degrees.of(shooterAngle)));
        }
    }

    protected void exhaustGamePieceFromClaw() {
        coralLocation = Optional.empty();
        gamePieceInClaw = false;
        double wristAngleRad = container.getWrist().getCurrentPosition();
        double projectileHeight =
                container.getRobotState().getElevatorHeightMeters()
                        + Constants.WristConstants.kWristZStartingOffset
                        - Math.cos(wristAngleRad) * Constants.WristConstants.kWristLength;
        double shooterAngle =
                new Rotation2d(wristAngleRad).rotateBy(Rotation2d.fromDegrees(-35)).getRadians();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(
                        new ReefscapeCoralOnFly(
                                this.simDrive.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(SimConstants.kProjectileOffsetXL1, 0),
                                this.simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                this.simDrive.getSimulatedDriveTrainPose().getRotation(),
                                Units.Meters.of(projectileHeight),
                                Units.MetersPerSecond.of(SimConstants.kCoralProjectileSpeed),
                                Units.Degrees.of(shooterAngle)));
    }

    protected void handleCoralIntakeSimulation() {
        double rollerRPS = container.getRobotState().getIntakeRollerRPS();
        double indexerRollerRPS = container.getRobotState().getIndexerRPS();

        // Get current pivot position from the pivot subsystem
        double pivotPosition = container.getRobotState().getIntakePivotRadians();
        boolean isPivotDeployed = Util.epsilonEquals(pivotPosition, 0, 0.1); // 0.1 rad tolerance

        double coralRollerRPS = container.getRobotState().getClawRollerRPS();
        Logger.recordOutput("Sim/CoralRollerRPS", coralRollerRPS);

        if (Math.abs(coralRollerRPS) > SimConstants.kIntakeRPSThreshold
                && container.getModalControls().getMode() == ModalControls.Mode.CORALMANUAL
                && gamePieceInClaw) {

            if (!coralExhaustTimerActive) {
                coralExhaustStartTime = Timer.getFPGATimestamp();
                coralExhaustTimerActive = true;
            }
            if (Timer.getFPGATimestamp() - coralExhaustStartTime
                    >= Constants.SimConstants.kRequiredExhaustTime) {
                exhaustGamePieceFromClaw();
                coralExhaustTimerActive = false;
            }
        } else {
            coralExhaustTimerActive = false;
        }

        // Only allow intake when pivot is deployed and voltage is applied
        if ((rollerRPS > SimConstants.kIntakeRPSThreshold && isPivotDeployed)
                || (isAtHPIntakeStation(
                                        getLatestFieldToRobot(),
                                        SimConstants.kHPIntakeDistanceThreshold)
                                && Math.abs(indexerRollerRPS) > SimConstants.kIntakeRPSThreshold)
                        && coralState == CoralState.NO_CORAL) {
            coralIntakeSimulation.startIntake();

            // Check if the intake was just started and we're at an HP station
            if (kForceCoralAlways
                    && coralLocation.isEmpty()
                    && isAtHPIntakeStation(
                            getLatestFieldToRobot(), SimConstants.kHPIntakeDistanceThreshold)) {
                inFunnelMode = true;
            } else if (coralLocation.isEmpty() && kForceCoralAlways) {
                if (DriverStation.isAutonomous()
                        && !isAtHPIntakeStation(
                                getLatestFieldToRobot(),
                                SimConstants.kHPIntakeDistanceThreshold * 2.0)) {
                    return;
                }
                coralLocation = Optional.of(0.0);
            }
        } else {
            coralIntakeSimulation.stopIntake();
        }

        // Detect when we have a coral in robot.
        if (coralIntakeSimulation.getGamePiecesAmount() > 0) {
            coralLocation = Optional.of(0.0);
            coralIntakeSimulation.obtainGamePieceFromIntake();
        }
    }

    protected Optional<Pose3d> computeCoralInRobotPose(
            Pose2d robotPose, Pose3d clawPose, double dt) {
        if (coralLocation.isEmpty() && !inFunnelMode) {
            if (gamePieceInClaw) {
                return Optional.of(calculateGamePiecePositionInClaw(robotPose, clawPose));
            } else {
                return Optional.empty();
            }
        }

        if (inFunnelMode) {
            return Optional.of(calculateCoralPositionInFunnel(robotPose, clawPose, dt));
        }

        // Regular intake/indexer code remains unchanged
        double location = coralLocation.get();

        double slipFactor = 0.2;
        if (location < intakeDistanceM) {
            // Tick intake in m/s
            double intakeLinearSpeed =
                    this.container.getRobotState().getIntakeRollerRPS()
                            * Constants.IntakeConstants.kIntakeRollerRadius
                            * 2.0
                            * Math.PI
                            * dt;
            location += intakeLinearSpeed * slipFactor;
        } else if (location < intakeDistanceM + indexerDistanceM) {
            // Tick indexer in m/s
            double indexerLinearSpeed =
                    this.container.getRobotState().getIndexerRPS()
                            * Constants.IndexerConstants.kIndexerRadius
                            * 2.0
                            * Math.PI
                            * dt;
            location += indexerLinearSpeed * slipFactor;
        }

        // When jamming, coral never leaves intake.
        if (kForceCoralJam) {
            location = Math.min(location, intakeDistanceM - 0.01);
            if (location < -indexerDistanceM) {
                coralLocation = Optional.empty();
                return Optional.empty();
            }
        }

        coralLocation = Optional.of(location);

        Pose3d coralPose = new Pose3d(robotPose);
        Transform3d toBeginningPose =
                new Transform3d(-0.5, 0.0, 0.2, new Rotation3d(0.0, Math.toRadians(-30.0), 0.0));
        Transform3d intakeTransform =
                new Transform3d(Util.limit(location, intakeDistanceM), 0.0, 0.0, Rotation3d.kZero);
        Transform3d turningTransform =
                new Transform3d(
                        Translation3d.kZero,
                        new Rotation3d(
                                0.0,
                                Math.toRadians(30.0 * Util.limit(location / intakeDistanceM, 1.0)),
                                0.0));
        coralPose = coralPose.plus(toBeginningPose).plus(intakeTransform).plus(turningTransform);

        // Location < intakeDistance no more rendering needed.
        if (location <= intakeDistanceM) return Optional.of(coralPose);

        Transform3d indexerTransform =
                new Transform3d(
                        location - intakeDistanceM, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
        coralPose = coralPose.plus(indexerTransform);
        if (location <= intakeDistanceM + indexerDistanceM) return Optional.of(coralPose);

        // Otherwise, we have reached the end of the linear path.
        coralLocation = Optional.empty();
        gamePieceInClaw = true;
        return Optional.of(calculateGamePiecePositionInClaw(robotPose, clawPose));
    }

    protected Pose3d calculateCoralPositionInFunnel(Pose2d robotPose, Pose3d clawPose, double dt) {
        inFunnelMode = false;
        // When funnel is complete, coral goes to middle of the indexer path
        coralLocation = Optional.of(intakeDistanceM + (indexerDistanceM / 2));
        return computeCoralInRobotPose(robotPose, clawPose, 0).get();
    }

    public void updateCoralState() {
        // If we're in funnel mode, maintain that state
        if (inFunnelMode) {
            coralState = CoralState.CORAL_IN_FUNNEL;
            return;
        }

        // Otherwise, determine state based on location
        if (coralLocation.isPresent()) {
            // adding 0.01 to account for slip factor
            if (coralLocation.get() < intakeDistanceM + 0.01) {
                coralState = CoralState.CORAL_IN_INTAKE;
            } else {
                coralState = CoralState.CORAL_IN_INDEXER;
            }
        } else {
            coralState = gamePieceInClaw ? CoralState.CORAL_IN_CLAW : CoralState.NO_CORAL;
        }
    }

    public void updateSensors() {
        container.getSimulatedClawSensors().setStageCoralBannerUntriggered();
        container.getSimulatedIndexerSensors().setFirstIndexerBannerUntriggered();
        container.getSimulatedIndexerSensors().setSecondIndexerBannerUntriggered();

        switch (coralState) {
            case CORAL_IN_INTAKE -> container
                    .getSimulatedIndexerSensors()
                    .setFirstIndexerBannerTriggered();
            case CORAL_IN_INDEXER -> {
                container.getSimulatedIndexerSensors().setSecondIndexerBannerTriggered();
                readyForHandoff = true;
            }
            case CORAL_IN_FUNNEL -> {
                container.getSimulatedIndexerSensors().setSecondIndexerBannerTriggered();
                readyForHandoff = true;
            }
            case CORAL_IN_CLAW -> {
                container.getSimulatedClawSensors().setStageCoralBannerTriggered();
            }
            default -> {}
        }

        // Update elevator hall effect
        container.getSimulatedElevatorSensors().setElevatorHallEffectUntriggered();
        if (container.getRobotState().getElevatorHeightMeters()
                < ElevatorConstants.kElevatorToleranceMeters) {
            container.getSimulatedElevatorSensors().setElevatorHallEffectTriggered();
        }

        updateToFSensors();

        // Updates for hasAlgae
        if (algaeInRobot) {
            container.getSimulatedClawMotor().overrideRPS(Optional.of(0.0));
        } else {
            container.getSimulatedClawMotor().overrideRPS(Optional.empty());
        }

        if (container.getRobotState().getLatestFieldToRobot().getValue().getX() > 7
                && container.getRobotState().getLatestFieldToRobot().getValue().getX() < 11
                && container.getClimberPivot().getCurrentPosition() > Math.toRadians(90)
                && container.getClimberRoller().getCurrentVelocity() > 0.1) {
            container.getSimulatedClimberSensors().setLeftClimberLimitSwitchTriggered();
            container.getSimulatedClimberSensors().setRightClimberLimitSwitchTriggered();
        }

        // simulate score banner being triggered by indexer at stow
        if (container.getElevator().getCurrentPosition() < 0.05
                && container.getWrist().getCurrentPosition() < Math.toRadians(-95)) {
            container.getSimulatedClawSensors().setScoreCoralBannerTriggered();
        } else {
            container.getSimulatedClawSensors().setScoreCoralBannerUntriggered();
        }
    }

    private Pose2d getClosestFace() {
        int closestFaceIndex = 0;
        double[] distances = new double[6];
        var robotPose =
                this.container.getRobotState().getLatestFieldToRobot().getValue().getTranslation();
        Pose2d closestFace = FieldConstants.Reef.centerFaces[0];

        for (int i = 0; i <= 5; i++) {
            Pose2d reefPose =
                    this.container.getRobotState().isRedAlliance()
                            ? FlippingUtil.flipFieldPose(FieldConstants.Reef.centerFaces[i])
                            : FieldConstants.Reef.centerFaces[i];
            distances[i] = robotPose.getDistance(reefPose.getTranslation());
            if (distances[i] < distances[closestFaceIndex]) {
                closestFaceIndex = i;
                closestFace = reefPose;
            }
        }

        return closestFace;
    }

    protected void updateToFSensors() {
        // Naively determine this computation to the closest reef face only.
        Pose2d closestFace = getClosestFace();
        Pose2d robotPose = this.container.getRobotState().getLatestFieldToRobot().getValue();

        // First, determine if this will even hit the closestFace based on robot rotation.
        // For simplicity, model this from front of robot and the exact corners.
        double diff = Constants.kRobotWidth / 2.0;
        Pose2d leftToF = robotPose.transformBy(new Transform2d(diff, diff, Rotation2d.kZero));
        Pose2d rightToF = robotPose.transformBy(new Transform2d(diff, -diff, Rotation2d.kZero));

        Optional<Double> leftDistance = computeToFDistance(leftToF, closestFace);
        Optional<Double> rightDistance = computeToFDistance(rightToF, closestFace);

        this.container
                .getSimulatedElevatorSensors()
                .setElevatorDistance(leftDistance.orElse(20.0), rightDistance.orElse(20.0));
    }

    protected Optional<Double> computeToFDistance(Pose2d tof, Pose2d face) {
        double faceSize = 0.960628; // in m
        Pose2d leftFacePoint =
                face.transformBy(new Transform2d(0.0, 0.0, Rotation2d.kCW_90deg))
                        .transformBy(new Transform2d(faceSize / 2.0, 0.0, Rotation2d.kZero));
        Pose2d rightFacePoint =
                face.transformBy(new Transform2d(0.0, 0.0, Rotation2d.kCCW_90deg))
                        .transformBy(new Transform2d(faceSize / 2.0, 0.0, Rotation2d.kZero));

        var intersectionPoint =
                GeometryUtil.intersectionPoint(
                        tof.getTranslation(),
                        tof.getRotation(),
                        leftFacePoint.getTranslation(),
                        rightFacePoint.getTranslation());
        if (intersectionPoint.isEmpty()) return Optional.empty();

        double distance = intersectionPoint.get().getDistance(tof.getTranslation());
        // Cap distance.
        if (distance > 1.5) distance = 20.0;
        return Optional.of(distance);
    }

    @SuppressWarnings("unused")
    protected void handleAlgaeIntakeSimulation() {
        double rollerRPS = this.container.getRobotState().getClawRollerRPS();

        if (coralState == CoralState.CORAL_IN_CLAW) return;

        // Check if roller is running at sufficient speed
        if (this.container.getModalControls().algaeClimbMode().getAsBoolean()
                && rollerRPS > SimConstants.kAlgaeIntakeRPSThreshold
                // Ground intake
                && (this.container.getStateMachine().getCurrentState()
                                == SuperstructureState.GROUND_ALGAE_INTAKE
                        || this.container.getStateMachine().getCurrentState()
                                == SuperstructureState.LOLLIPOP_INTAKE
                        // Or reef intake and close to reef
                        || ((this.container.getStateMachine().getCurrentState()
                                                == SuperstructureState.REEF_ALGAE_INTAKE_L2
                                        || this.container.getStateMachine().getCurrentState()
                                                == SuperstructureState.REEF_ALGAE_INTAKE_L3)
                                && !ReefscapeUtil.awayFromReef(this.container)
                                && extraCloseToReef(this.container)))) {
            algaeIntakeSimulation.startIntake();
            if (!algaeInRobot && kForceAlgaeAlways) {
                algaeInRobot = true;
            }
        } else {
            algaeIntakeSimulation.stopIntake();
        }

        if (algaeIntakeSimulation.getGamePiecesAmount() > 0) {
            algaeInRobot = true;
            algaeIntakeSimulation.obtainGamePieceFromIntake();
        }
    }

    // For sim auto purposes to ensure no auto stowing before the algae is acquired
    public static boolean extraCloseToReef(RobotContainer container) {
        Translation2d reefCenter =
                container.getRobotState().isRedAlliance()
                        ? Util.flipRedBlue(FieldConstants.Reef.center)
                        : FieldConstants.Reef.center;

        Pose2d robotPose = container.getRobotState().getLatestFieldToRobot().getValue();
        double distance = robotPose.getTranslation().getDistance(reefCenter);
        double buffer = 0.0;
        return distance < Constants.kReefRadius + Constants.kRobotWidth / 2.0 + buffer;
    }

    protected Optional<Pose3d> computeAlgaeInRobotPose(
            Pose2d robotPose, Pose3d clawPose, double dt) {
        if (!algaeInRobot) {
            return Optional.empty();
        }
        return Optional.of(calculateGamePiecePositionInClaw(robotPose, clawPose));
    }

    public void updateAlgaeState() {
        algaeState = algaeInRobot ? AlgaeState.ALGAE_IN_WRIST : AlgaeState.NO_ALGAE;
    }

    protected void scoreAlgaeSim(Pose3d algaeInRobotPose) {
        double clawAngle = container.getWrist().getCurrentPosition();
        double releaseAngle = clawAngle;
        // Score the algae
        algaeInRobot = false;
        Translation2d robotTranslation2d =
                this.simDrive.getSimulatedDriveTrainPose().getTranslation();
        Translation2d relativePosition =
                algaeInRobotPose
                        .getTranslation()
                        .toTranslation2d()
                        .minus(robotTranslation2d)
                        .rotateBy(
                                this.simDrive
                                        .getSimulatedDriveTrainPose()
                                        .getRotation()
                                        .unaryMinus());

        ReefscapeAlgaeOnFly algaeProjectile =
                new ReefscapeAlgaeOnFly(
                        robotTranslation2d,
                        relativePosition,
                        this.simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        this.simDrive.getSimulatedDriveTrainPose().getRotation(),
                        Units.Meters.of(algaeInRobotPose.getZ()),
                        Units.MetersPerSecond.of(SimConstants.kAlgaeProjectileSpeed),
                        Units.Radians.of(releaseAngle));

        algaeProjectile.withProjectileTrajectoryDisplayCallBack(
                (poses) -> {
                    Logger.recordOutput(
                            "Sim/AlgaeScoring/SuccessfulTrajectory", poses.toArray(Pose3d[]::new));
                },
                (poses) -> {
                    Logger.recordOutput(
                            "Sim/AlgaeScoring/MissedTrajectory", poses.toArray(Pose3d[]::new));
                });

        SimulatedArena.getInstance().addGamePieceProjectile(algaeProjectile);
    }

    protected boolean isAtHPIntakeStation(Pose2d robotPose, double threshold) {
        if (robotPose == null) return false;

        // Check for either HP station A or B
        boolean atStationA =
                isNearHPStation(
                        robotPose,
                        this.container.getRobotState().isRedAlliance()
                                ? Util.flipRedBlue(FieldConstants.HPIntake.kStationA)
                                : FieldConstants.HPIntake.kStationA,
                        threshold);
        boolean atStationB =
                isNearHPStation(
                        robotPose,
                        this.container.getRobotState().isRedAlliance()
                                ? Util.flipRedBlue(FieldConstants.HPIntake.kStationB)
                                : FieldConstants.HPIntake.kStationB,
                        threshold);

        return atStationA || atStationB;
    }

    protected boolean isNearHPStation(Pose2d robotPose, Pose2d station, double threshold) {
        double distance = robotPose.getTranslation().getDistance(station.getTranslation());
        if (distance > threshold) {
            return false;
        }
        return true;
    }

    public void initializeWithCoral() {
        coralState = CoralState.CORAL_IN_CLAW;
        gamePieceInClaw = true;
        container.getSimulatedClawSensors().setStageCoralBannerTriggered();
    }

    public void updateClimberPivot(double dt) {
        // Constrain the climber between [0, 97].
        var originalPos = this.container.getClimberPivot().getCurrentPosition();
        var velocity = this.container.getClimberPivot().getCurrentVelocity();
        var pos =
                Math.min(
                        originalPos, Constants.ClimberConstants.kClimberPivotDeployPositionRadians);
        if (climberState == ClimberState.DEPLOYING) {
            if (pos != originalPos) {
                this.container.getSimulateClimberPivot().setPositionRad(pos);
                this.container
                        .getSimulateClimberPivot()
                        .overridePos(
                                Optional.of(
                                        Constants.ClimberConstants
                                                .kClimberPivotDeployPositionRadians));
                climberState = ClimberState.IN_SLACK;
            }
        } else if (climberState == ClimberState.IN_SLACK) {
            this.container.getSimulateClimberPivot().setPositionRad(pos);
            // If we are trying to go past, then run for a bit more.
            climberUnspoolDistance += velocity * dt;
            if (climberUnspoolDistance > 0.5) {
                climberState = ClimberState.STOWING;
                this.container.getSimulateClimberPivot().overridePos(Optional.empty());
            }
        } else if (climberState == ClimberState.STOWING) {
            this.container.getSimulateClimberPivot().setInvertVoltage(true);
            pos = Math.max(originalPos, 0.0);
            if (pos != originalPos) {
                this.container.getSimulateClimberPivot().setPositionRad(pos);
                this.container.getSimulateClimberPivot().overridePos(Optional.of(pos));
            }
        }

        Logger.recordOutput("Sim/ClimberUnspoolDistance", climberUnspoolDistance);
        Logger.recordOutput("Sim/ClimberState", climberState);
    }

    public synchronized void updateSim() {
        handleCoralIntakeSimulation();
        handleAlgaeIntakeSimulation();
        double delta = Timer.getFPGATimestamp() - lastTimestamp;

        var robotPose = simDrive.getSimulatedDriveTrainPose();
        var clawPose = container.getRobotViz().getClawPose3d();

        updateClimberPivot(delta);

        Optional<Pose3d> coralInRobotPose = computeCoralInRobotPose(robotPose, clawPose, delta);
        if (coralInRobotPose.isPresent()) {
            Logger.recordOutput("Sim/HeldCoral/Pose3d", coralInRobotPose.get());
        } else {
            Logger.recordOutput("Sim/HeldCoral/Pose3d", SimConstants.kInvalidGamePiecePose);
        }

        Logger.recordOutput("Sim/CoralInFunnel", inFunnelMode);

        Optional<Pose3d> algaeInRobotPose = computeAlgaeInRobotPose(robotPose, clawPose, delta);
        double algaeRollerRPS = this.container.getSimulatedClawMotor().getIntendedRPS();

        if (algaeInRobot && -algaeRollerRPS > SimConstants.kIntakeRPSThreshold) {
            scoreAlgaeSim(algaeInRobotPose.get());
        }

        Logger.recordOutput(
                "Sim/HeldAlgae/Pose3d",
                algaeInRobotPose.orElse(SimConstants.kInvalidGamePiecePose));

        if (gamePieceInClaw) {
            // Check for branch contact
            FieldConstants.ReefHeight contactHeight =
                    calculateBranchContact(coralInRobotPose.get());
            if (contactHeight != null
                    && container.getStateMachine().getCurrentState()
                            != SuperstructureState.STAGE_CORAL_L1) {
                scoreCoralSim(contactHeight);
            }

            checkL1Coral();
        }

        if (coralLocation.isPresent()) {
            Logger.recordOutput("Sim/CoralLinear", coralLocation.get());
        } else {
            Logger.recordOutput("Sim/CoralLinear", -1.0);
        }

        lastTimestamp = Timer.getFPGATimestamp();
        // Update states
        updateCoralState();
        updateAlgaeState();
        updateSensors();

        // Log states
        Logger.recordOutput("Sim/CoralState", coralState.toString());
        Logger.recordOutput("Sim/AlgaeState", algaeState.toString());
    }
}
