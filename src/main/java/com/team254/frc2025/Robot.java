// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2025;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.team254.frc2025.auto.AutoModeSelector.DesiredMode;
import com.team254.frc2025.auto.AutoModeSelector.FeederStrategy;
import com.team254.frc2025.auto.AutoModeSelector.StartingPosition;
import com.team254.frc2025.auto.PathfindingAuto;
import com.team254.frc2025.auto.PathfindingWarmupCommand;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team254.lib.pathplanner.util.FlippingUtil;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.CANBusStatusLogger;
import com.team254.lib.util.CANStatusLogger;
import com.team254.lib.util.OSUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    static final int kRTPriority = 2;
    static final int kNonRTPriority = 1;

    private Command disabledCommand = Commands.none();
    private boolean hasEnabled = false;

    private final RobotContainer robotContainer;
    private int mIter = 0;
    private Command autonomousCommand = Commands.none();
    private Optional<Pose2d> startingPose = Optional.empty();

    private DesiredMode desiredMode = null;
    private String scoringSequence = "";
    private String levelSequence = "";
    private StartingPosition startingLocation = null;
    private FeederStrategy feederStrategy = null;
    private Integer iceCreamCnt = null;
    private Optional<Alliance> allianceColor = Optional.of(Alliance.Blue);
    private LoggedNetworkString latestProcessedScoreOrder =
            new LoggedNetworkString("[Check]LatestProcessedScoreOrder", "Not Set");
    private double lastTimestampNotValid = 0;

    private double timeOfLastSync = 0.0;

    private Command warmupCommand;
    private PathfindingWarmupCommand pathfindingWarmupCommand;
    private CANBusStatusLogger driverCAN =
            new CANBusStatusLogger(Constants.kCanBusDrivebaseClimberCanivore);
    private CANBusStatusLogger superstructureCAN =
            new CANBusStatusLogger(Constants.kCanBusSuperstructureCanivore);

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            if (!DriverStation.isFMSAttached()) {
                Logger.addDataReceiver(new NT4Publisher());
            }
        } else if (Constants.kIsReplay) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else if (RobotBase.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        }

        Logger.start();
        if (!Logger.hasReplaySource()) {
            RobotController.setTimeSource(RobotController::getFPGATime);
        }

        robotContainer = new RobotContainer();
        if (RobotBase.isSimulation()) {
            robotContainer.getDriveSubsystem().resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        }
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SignalLogger.enableAutoLogging(false);

        warmupCommand = PathfindingCommand.warmupCommand();
        warmupCommand.schedule();
    }

    @Override
    public void robotPeriodic() {
        if (DriverStation.isEnabled()) {
            Threads.setCurrentThreadPriority(true, kRTPriority);
        } else {
            Threads.setCurrentThreadPriority(false, kNonRTPriority);
        }
        CommandScheduler.getInstance().run();
        robotContainer.getClaw().updateWristRPS(robotContainer.getWrist().getCurrentVelocity());
        robotContainer.getClaw().updateMode(robotContainer.getModalControls().getMode());

        robotContainer.getRobotState().updateLogger();
        robotContainer.getRobotViz().updateViz();
        if (Robot.isSimulation()) {
            robotContainer.getSimulatedRobotState().updateSim();
        }
        Logger.recordOutput(
                "Current_Superstructure_State",
                robotContainer.getStateMachine().getCurrentState() == null
                        ? "null"
                        : robotContainer.getStateMachine().getCurrentState().toString());
        Logger.recordOutput(
                "Desired_Superstructure_State",
                robotContainer.getStateMachine().getDesiredState() == null
                        ? "null"
                        : robotContainer.getStateMachine().getDesiredState().toString());
        Logger.recordOutput(
                "Future_Desired_Superstructure_State",
                robotContainer.getStateMachine().getFutureDesiredState() == null
                                || robotContainer
                                                .getStateMachine()
                                                .getFutureDesiredState()
                                                .toString()
                                        == null
                        ? "null"
                        : robotContainer.getStateMachine().getFutureDesiredState().toString());
        Logger.recordOutput(
                "Current_Coral_State",
                robotContainer.getCoralStateTracker().getCurrentPosition() == null
                        ? "null"
                        : robotContainer.getCoralStateTracker().getCurrentPosition().toString());
        Logger.recordOutput(
                "Current_Coral_State", robotContainer.getCoralStateTracker().getCurrentPosition());

        Logger.recordOutput(
                "Default_Robot_Wide",
                robotContainer.getModalSuperstructureTriggers().getDefaultRobotWide().get());
        Threads.setCurrentThreadPriority(false, kNonRTPriority);
    }

    @Override
    public void disabledInit() {
        timeOfLastSync = Timer.getFPGATimestamp();
        disabledCommand = robotContainer.getDisabledCommand();
        disabledCommand.schedule();
        Pathfinding.ensureInitialized();
        Pathfinding.setTeleopObstacles(); // using teleop obstacles since we want that for auto
        Pathfinding.enableCaching();
        Pathfinding.setCacheDistanceToleranceMeters(0.0);

        if (robotContainer.getClimberPivot().getClimberDeployDone().get()) {
            Commands.waitSeconds(5.0)
                    .andThen(
                            new InstantCommand(
                                    () ->
                                            robotContainer
                                                    .getClimberPivot()
                                                    .setNeutralMode(NeutralModeValue.Coast)))
                    .ignoringDisable(true)
                    .schedule();
        }

        robotContainer
                .getDriveSubsystem()
                .getController()
                .accept(PathPlannerTrajectory.makeStayInPlaceTrajectory());
    }

    @Override
    public void disabledPeriodic() {

        SmartDashboard.putNumber("WristRadians", robotContainer.getWrist().getCurrentPosition());
        if (hasEnabled && (Timer.getFPGATimestamp() - timeOfLastSync) >= 10.0) {
            OSUtil.fsSyncAsync();
            timeOfLastSync = Timer.getFPGATimestamp();
        }
        if (mIter % 50 == 0) {
            NetworkTableInstance.getDefault().flush();
            robotContainer.getClaw().setCurrentPosition(0.0);
            if (!hasEnabled) {
                robotContainer.getIntakePivot().resetOffset();
            }

            if (robotContainer.isValidAutoCommand()) {
                SmartDashboard.putNumber(
                        "Time Since Auto Command Not Valid",
                        RobotTime.getTimestampSeconds() - lastTimestampNotValid);
            } else {
                SmartDashboard.putNumber("Time Since Auto Command Not Valid", 0);
                lastTimestampNotValid = RobotTime.getTimestampSeconds();
            }
            if (startingPose.isPresent()
                    && robotContainer.odometryCloseToPose(startingPose.get())) {
                SmartDashboard.putBoolean("Near Auto Starting Pose", true);
            } else {
                SmartDashboard.putBoolean("Near Auto Starting Pose", false);
            }
            DesiredMode latestDesiredMode = robotContainer.getModeChooser().get();
            String latestScoringSequence = robotContainer.getScoringSequence();
            String latestLevelSequence = robotContainer.getLevelSequence();
            StartingPosition latestStartingLocation =
                    robotContainer.getStartingPositionChooser().get();
            Integer latestIceCreamCnt = robotContainer.getIceCreamCountChooser().get();
            FeederStrategy latestFeederStrategy = robotContainer.getFeederStrategyChooser().get();

            if (desiredMode != latestDesiredMode
                    || startingLocation != latestStartingLocation
                    || feederStrategy != latestFeederStrategy
                    || !latestScoringSequence.equals(scoringSequence)
                    || !levelSequence.equals(latestLevelSequence)
                    || latestIceCreamCnt != iceCreamCnt
                    || (DriverStation.getAlliance().isPresent()
                            && allianceColor.get() != DriverStation.getAlliance().get())) {
                if (latestScoringSequence.length() != latestLevelSequence.length()) {
                    System.out.println("Unequal lenths of sequence for level and scoring.");
                    mIter++;
                    return;
                }
                autonomousCommand = robotContainer.getAutonomousCommand();
                if (autonomousCommand instanceof PathfindingAuto) {
                    startingPose = ((PathfindingAuto) autonomousCommand).getStartingPose();
                    if (startingPose.isPresent()) {
                        if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get()
                                        == DriverStation.Alliance.Red) {
                            startingPose =
                                    Optional.of(FlippingUtil.flipFieldPose(startingPose.get()));
                        }
                        robotContainer.getDriveSubsystem().resetOdometry(startingPose.get());
                    }
                    pathfindingWarmupCommand =
                            new PathfindingWarmupCommand(
                                    robotContainer,
                                    latestScoringSequence,
                                    latestLevelSequence,
                                    latestStartingLocation,
                                    0,
                                    latestFeederStrategy,
                                    () -> robotContainer.getRobotState().isRedAlliance());
                    warmupCommand = pathfindingWarmupCommand.getWarmupCommand();
                    warmupCommand.schedule();
                }
                allianceColor =
                        DriverStation.getAlliance().isPresent()
                                ? DriverStation.getAlliance()
                                : allianceColor;
                desiredMode = latestDesiredMode;
                startingLocation = latestStartingLocation;
                feederStrategy = latestFeederStrategy;
                iceCreamCnt = latestIceCreamCnt;
                scoringSequence = latestScoringSequence;
                levelSequence = latestLevelSequence;
                robotContainer.setAutoDefaultCommands(feederStrategy);
                Logger.recordOutput(
                        "Regenerating Auto Command Timestamp", Timer.getFPGATimestamp());
                Logger.recordOutput("Regenerating Auto Command", autonomousCommand.getName());
            }
            if (autonomousCommand != null
                    && autonomousCommand instanceof PathfindingAuto
                    && DriverStation.getAlliance().isPresent()) {
                latestProcessedScoreOrder.set(robotContainer.getScoringSequence());
                System.out.println(
                        "Score Order: "
                                + robotContainer.getScoringSequence()
                                + " Level Order: "
                                + robotContainer.getLevelSequence());
                // Also ensure we have warmed up the path from the actual starting pose
                // periodically.
                if (warmupCommand != null
                        && !warmupCommand.isScheduled()
                        && pathfindingWarmupCommand != null) {
                    System.out.println("warmup command scheduling!");
                    warmupCommand =
                            pathfindingWarmupCommand.getFirstWarmupCommand(
                                    robotContainer
                                            .getRobotState()
                                            .getLatestFieldToRobot()
                                            .getValue());
                    warmupCommand.schedule();
                } else {
                    System.out.println("Warmup command already scheduled!");
                }
            }
            CANStatusLogger.getInstance().updateCanStatus();
        }
        mIter++;

        driverCAN.logStatus();
        superstructureCAN.logStatus();
    }

    @Override
    public void disabledExit() {
        disabledCommand.cancel();
        warmupCommand.cancel();

        robotContainer.getVisionSubsystem().setUseVision(true);

        // This clears the tablesToData map in SmartDashboard, so it stops trying to update
        // anything.
        SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
    }

    @Override
    public void autonomousInit() {
        Threads.setCurrentThreadPriority(true, kRTPriority);

        Pathfinding.ensureInitialized();
        Pathfinding.setCacheDistanceToleranceMeters(0.8);

        robotContainer.getModalSuperstructureTriggers().getDefaultRobotWide().set(true);
        if (robotContainer.getLevelSequence().length() >= 1
                && robotContainer.getLevelSequence().charAt(0) != '*')
            robotContainer.getCoralStateTracker().forceSet(CoralPosition.STAGED_IN_CLAW);
        if (Robot.isSimulation()) {
            if (robotContainer.getLevelSequence().length() >= 1
                    && robotContainer.getLevelSequence().charAt(0) != '*')
                robotContainer.getSimulatedRobotState().initializeWithCoral();
            if (!hasEnabled) {
                SimulatedArena.getInstance().placeGamePiecesOnField();
            }
        }
        if (!hasEnabled) {
            hasEnabled = true;
        }

        robotContainer.getRobotState().setAutoStartTime(Timer.getFPGATimestamp());

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void robotInit() {
        super.robotInit();
        if (Robot.isSimulation()) {
            robotContainer.getSimulatedWristMotor().setPositionRad(Units.degreesToRadians(-18.0));
        }
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.getRobotState().logControllerMode();
    }

    @Override
    public void autonomousExit() {
        robotContainer.getModalSuperstructureTriggers().getDefaultRobotWide().set(true);
    }

    @Override
    public void teleopInit() {
        Threads.setCurrentThreadPriority(true, kRTPriority);
        // This clears the tablesToData map in SmartDashboard, so it stops trying to update
        // anything.
        SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());

        if (Robot.isSimulation() && !hasEnabled) {
            SimulatedArena.getInstance().placeGamePiecesOnField();
            // Add a coral piece to make life easier in sim.
            SimulatedArena.getInstance()
                    .addGamePiece(new ReefscapeCoralOnField(new Pose2d(2, 2, Rotation2d.kZero)));
        }

        robotContainer
                .getClimberPivot()
                .setRawPositionOffset(robotContainer.getClimberPivot().getRawRotorPosition());

        if (!hasEnabled) {
            hasEnabled = true;
            robotContainer
                    .getModalSuperstructureTriggers()
                    .setStateAndWaitCommand(
                            SuperstructureState.STOW_CORAL, "Stow Coral Teleop Init")
                    .schedule();
        }

        Pathfinding.ensureInitialized();
        Pathfinding.disableCaching();
        Pathfinding.setTeleopObstacles();

        if (robotContainer.getClaw().hasCoralAtStageBanner()) {
            robotContainer.getModalSuperstructureTriggers().setCheckIfCoralAtStageBanner(true);
        } else {
            robotContainer.getModalSuperstructureTriggers().setCheckIfCoralAtStageBanner(false);
        }

        robotContainer.setTeleopDefaultCommands();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        if (Robot.isSimulation()) {

            SimulatedArena.getInstance().placeGamePiecesOnField();
            // Add a coral piece to make life easier in sim.
            SimulatedArena.getInstance()
                    .addGamePiece(new ReefscapeCoralOnField(new Pose2d(2, 2, Rotation2d.kZero)));
        }
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.getRobotState().logControllerMode();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
