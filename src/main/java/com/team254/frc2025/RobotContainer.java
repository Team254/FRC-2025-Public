// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2025;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.team254.frc2025.auto.AutoModeSelector;
import com.team254.frc2025.auto.AutoModeSelector.DesiredMode;
import com.team254.frc2025.auto.AutoModeSelector.FeederStrategy;
import com.team254.frc2025.auto.AutoModeSelector.StartingPosition;
import com.team254.frc2025.commands.DriveMaintainingHeadingCommand;
import com.team254.frc2025.controlboard.ControlBoard;
import com.team254.frc2025.controlboard.ModalControls;
import com.team254.frc2025.simulation.SimulatedRobotState;
import com.team254.frc2025.subsystems.claw.ClawSensorIOHardware;
import com.team254.frc2025.subsystems.claw.ClawSensorIOSim;
import com.team254.frc2025.subsystems.claw.ClawSubsystem;
import com.team254.frc2025.subsystems.climber.ClimberPivotSubsystem;
import com.team254.frc2025.subsystems.climber.ClimberRollerSubsystem;
import com.team254.frc2025.subsystems.climber.ClimberSensorIOHardware;
import com.team254.frc2025.subsystems.climber.ClimberSensorIOSim;
import com.team254.frc2025.subsystems.drive.DriveIOHardware;
import com.team254.frc2025.subsystems.drive.DriveIOSim;
import com.team254.frc2025.subsystems.drive.DriveSubsystem;
import com.team254.frc2025.subsystems.elevator.ElevatorSensorIOHardware;
import com.team254.frc2025.subsystems.elevator.ElevatorSensorIOSim;
import com.team254.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team254.frc2025.subsystems.indexer.IndexerSensorIOHardware;
import com.team254.frc2025.subsystems.indexer.IndexerSensorIOSim;
import com.team254.frc2025.subsystems.indexer.IndexerSubsystem;
import com.team254.frc2025.subsystems.intake.IntakePivotSubsystem;
import com.team254.frc2025.subsystems.intake.IntakeRollerSubsystem;
import com.team254.frc2025.subsystems.led.LedIOHardware;
import com.team254.frc2025.subsystems.led.LedSubsystem;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker;
import com.team254.frc2025.subsystems.superstructure.ModalSuperstructureTriggers;
import com.team254.frc2025.subsystems.superstructure.SuperstructureStateMachine;
import com.team254.frc2025.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.frc2025.subsystems.vision.VisionIOHardwareLimelight;
import com.team254.frc2025.subsystems.vision.VisionIOSimPhoton;
import com.team254.frc2025.subsystems.vision.VisionSubsystem;
import com.team254.frc2025.subsystems.wrist.WristSubsystem;
import com.team254.frc2025.viz.ReefViz;
import com.team254.frc2025.viz.RobotViz;
import com.team254.lib.subsystems.*;
import com.team254.lib.util.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private DriveSubsystem buildDriveSystem() {
        if (RobotBase.isSimulation()) {
            return new DriveSubsystem(
                    new DriveIOSim(
                            robotState,
                            simulatedRobotState,
                            Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                    robotState);
        } else {
            return new DriveSubsystem(
                    new DriveIOHardware(
                            robotState,
                            Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                    robotState);
        }
    }

    private VisionSubsystem buildVisionSystem() {
        if (RobotBase.isSimulation()) {
            return new VisionSubsystem(
                    new VisionIOSimPhoton(robotState, simulatedRobotState), robotState);
        }
        return new VisionSubsystem(new VisionIOHardwareLimelight(robotState), robotState);
    }

    private ClawSubsystem buildClawSubsystem() {
        if (RobotBase.isSimulation()) {
            return new ClawSubsystem(
                    Constants.kClawConfig, simulatedClawMotor, simulatedClawSensors, this);
        }
        return new ClawSubsystem(
                Constants.kClawConfig,
                new TalonFXIO(Constants.kClawConfig),
                new ClawSensorIOHardware(),
                this);
    }

    public ClimberPivotSubsystem buildClimberPivotSubsystem() {
        if (RobotBase.isSimulation()) {
            return new ClimberPivotSubsystem(
                    Constants.kClimberPivotConfig,
                    simulatedClimberPivotMotor,
                    new SimCanCoderIO(
                            Constants.kClimberPivotConfig.canCoderConfig,
                            simulatedClimberPivotMotor.getSupplierForCancoder(
                                    Constants.kClimberPivotConfig)),
                    robotState);
        }
        return new ClimberPivotSubsystem(
                Constants.kClimberPivotConfig,
                new TalonFXIO(Constants.kClimberPivotConfig),
                new CanCoderIOHardware(Constants.kClimberPivotConfig.canCoderConfig),
                robotState);
    }

    private ClimberRollerSubsystem buildClimberRollerSubsystem() {
        if (RobotBase.isSimulation()) {
            return new ClimberRollerSubsystem(
                    Constants.kClimberRollerConfig,
                    new SimTalonFXIO(Constants.kClimberRollerConfig),
                    simulatedClimberSensors,
                    robotState);
        }
        return new ClimberRollerSubsystem(
                Constants.kClimberRollerConfig,
                new TalonFXIO(Constants.kClimberRollerConfig),
                new ClimberSensorIOHardware(),
                robotState);
    }

    private ElevatorSubsystem buildElevatorSubsystem() {
        assert Constants.kElevatorConfig.followers.length != 1 : "Expected length 1 for elevator";
        if (RobotBase.isSimulation()) {
            SimElevator simElevator =
                    new SimElevator(Constants.kElevatorConfig, Constants.kSimElevatorConfig);
            return new ElevatorSubsystem(
                    Constants.kElevatorConfig,
                    simElevator.getLeadIO(),
                    simElevator.getFollowerIO(),
                    simulatedElevatorSensors,
                    robotState);
        }
        return new ElevatorSubsystem(
                Constants.kElevatorConfig,
                new TalonFXIO(Constants.kElevatorConfig),
                new TalonFXIO[] {new TalonFXIO(Constants.kElevatorConfig.followers[0].config)},
                new ElevatorSensorIOHardware(),
                robotState);
    }

    public WristSubsystem buildWristSubsystem() {
        if (RobotBase.isSimulation()) {
            return new WristSubsystem(
                    Constants.kWristConfig,
                    simulatedWristMotor,
                    new SimCanCoderIO(
                            Constants.kWristConfig.canCoderConfig,
                            simulatedWristMotor.getSupplierForCancoder(Constants.kWristConfig)),
                    robotState);
        }
        return new WristSubsystem(
                Constants.kWristConfig,
                new TalonFXIO(Constants.kWristConfig),
                new CanCoderIOHardware(Constants.kWristConfig.canCoderConfig),
                robotState);
    }

    public IndexerSubsystem buildIndexerSubsystem() {
        if (RobotBase.isSimulation()) {
            return new IndexerSubsystem(
                    Constants.kIndexerConfig,
                    new SimTalonFXIO(Constants.kIndexerConfig),
                    simulatedIndexerSensors,
                    robotState,
                    coralStateTracker);
        }
        return new IndexerSubsystem(
                Constants.kIndexerConfig,
                new TalonFXIO(Constants.kIndexerConfig),
                new IndexerSensorIOHardware(),
                robotState,
                coralStateTracker);
    }

    public IntakeRollerSubsystem buildIntakeRollerSubsystem() {
        if (RobotBase.isSimulation()) {
            return new IntakeRollerSubsystem(
                    Constants.kIntakeRollerConfig,
                    new SimTalonFXIO(Constants.kIntakeRollerConfig),
                    robotState);
        }
        return new IntakeRollerSubsystem(
                Constants.kIntakeRollerConfig,
                new TalonFXIO(Constants.kIntakeRollerConfig),
                robotState);
    }

    public IntakePivotSubsystem buildIntakePivotSubsystem() {
        if (RobotBase.isSimulation()) {
            var simTalon = new SimTalonFXIO(Constants.kIntakePivotConfig);
            return new IntakePivotSubsystem(
                    Constants.kIntakePivotConfig,
                    simTalon,
                    new SimCanCoderIO(
                            Constants.kIntakePivotConfig.canCoderConfig,
                            simTalon.getSupplierForCancoder(Constants.kIntakePivotConfig)),
                    robotState);
        }
        return new IntakePivotSubsystem(
                Constants.kIntakePivotConfig,
                new TalonFXIO(Constants.kIntakePivotConfig),
                new CanCoderIOHardware(Constants.kIntakePivotConfig.canCoderConfig),
                robotState);
    }

    public LedSubsystem buildLedSubsystem() {
        return new LedSubsystem(new LedIOHardware(), robotState);
    }

    private final ControlBoard controlBoard = ControlBoard.getInstance();
    private final ModalControls modalControls = ModalControls.getInstance();
    private ModalSuperstructureTriggers modalSuperstructureTriggers;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer =
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate estimate) {
                    driveSubsystem.addVisionMeasurement(estimate);
                }
            };

    public ControlBoard getControlBoard() {
        return controlBoard;
    }

    private final RobotState robotState = new RobotState(visionEstimateConsumer);

    private final SuperstructureStateMachine stateMachine;
    private final CoralStateTracker coralStateTracker = new CoralStateTracker();
    private final SimulatedRobotState simulatedRobotState =
            Robot.isSimulation() ? new SimulatedRobotState(this) : null;
    private final IndexerSensorIOSim simulatedIndexerSensors =
            Robot.isSimulation() ? new IndexerSensorIOSim() : null;
    private final ClawSensorIOSim simulatedClawSensors =
            Robot.isSimulation() ? new ClawSensorIOSim() : null;
    private final ClimberSensorIOSim simulatedClimberSensors =
            Robot.isSimulation() ? new ClimberSensorIOSim() : null;
    private final ElevatorSensorIOSim simulatedElevatorSensors =
            Robot.isSimulation() ? new ElevatorSensorIOSim() : null;
    private final SimTalonFXIO simulatedClawMotor =
            Robot.isSimulation() ? new SimTalonFXIO(Constants.kClawConfig) : null;
    private final SimTalonFXWithCancoder simulatedWristMotor =
            Robot.isSimulation() ? new SimTalonFXWithCancoder(Constants.kWristConfig) : null;
    private final SimTalonFXWithCancoder simulatedClimberPivotMotor =
            Robot.isSimulation() ? new SimTalonFXWithCancoder(Constants.kClimberPivotConfig) : null;

    private final DriveSubsystem driveSubsystem = buildDriveSystem();

    private final VisionSubsystem visionSystem = buildVisionSystem();

    private final ClawSubsystem clawSubsystem = buildClawSubsystem();

    private final ClimberPivotSubsystem climberPivotSubsystem = buildClimberPivotSubsystem();

    private final ClimberRollerSubsystem climberRollerSubsystem = buildClimberRollerSubsystem();

    private final ElevatorSubsystem elevatorSubsystem = buildElevatorSubsystem();

    private final WristSubsystem wristSubsystem = buildWristSubsystem();

    private final IndexerSubsystem indexerSubsystem = buildIndexerSubsystem();

    private final IntakeRollerSubsystem intakeRollerSubsystem = buildIntakeRollerSubsystem();

    private final IntakePivotSubsystem intakePivotSubsystem = buildIntakePivotSubsystem();

    private final LedSubsystem ledSubsystem = buildLedSubsystem();

    private final ReefViz reefViz = ReefViz.getInstance();

    public ReefViz getReefViz() {
        return reefViz;
    }

    private final DriveMaintainingHeadingCommand driveCommand =
            (new DriveMaintainingHeadingCommand(
                    driveSubsystem,
                    robotState,
                    this,
                    controlBoard::getThrottle,
                    controlBoard::getStrafe,
                    controlBoard::getRotation));

    private final RobotViz robotViz = new RobotViz(robotState);

    private final AutoModeSelector autoModeSelector;

    public RobotContainer() {
        if (Robot.isSimulation()) {
            assert this.simulatedRobotState != null;
            this.simulatedRobotState.init();
        }
        stateMachine = new SuperstructureStateMachine(this);

        autoModeSelector = new AutoModeSelector(this);
        configureBindings();
        SmartDashboard.putBoolean("Is Practice Bot", Constants.kIsPracticeBot);
    }

    SwerveDriveBrake xWheels = new SwerveDriveBrake();

    private void configureBindings() {
        modalControls.configureBindings();
        modalSuperstructureTriggers =
                new ModalSuperstructureTriggers(this, stateMachine, coralStateTracker);
        driveSubsystem.setDefaultCommand(driveCommand);
    }

    public void resetHeading() {
        driveSubsystem.resetOdometry(
                new Pose2d(
                        new Translation2d(
                                robotState.getLatestFieldToRobot().getValue().getX(),
                                robotState.getLatestFieldToRobot().getValue().getY()),
                        robotState.isRedAlliance()
                                ? MathHelpers.kRotation2dPi
                                : MathHelpers.kRotation2dZero));
    }

    public ModalSuperstructureTriggers getModalSuperstructureTriggers() {
        return modalSuperstructureTriggers;
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSystem;
    }

    public ClawSubsystem getClaw() {
        return clawSubsystem;
    }

    public ClimberPivotSubsystem getClimberPivot() {
        return climberPivotSubsystem;
    }

    public ClimberRollerSubsystem getClimberRoller() {
        return climberRollerSubsystem;
    }

    public ElevatorSubsystem getElevator() {
        return elevatorSubsystem;
    }

    public WristSubsystem getWrist() {
        return wristSubsystem;
    }

    public IndexerSubsystem getIndexer() {
        return indexerSubsystem;
    }

    public IntakeRollerSubsystem getIntakeRoller() {
        return intakeRollerSubsystem;
    }

    public IntakePivotSubsystem getIntakePivot() {
        return intakePivotSubsystem;
    }

    public LedSubsystem getLeds() {
        return ledSubsystem;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public CoralStateTracker getCoralStateTracker() {
        return coralStateTracker;
    }

    public ModalControls getModalControls() {
        return modalControls;
    }

    public SimulatedRobotState getSimulatedRobotState() {
        return simulatedRobotState;
    }

    public IndexerSensorIOSim getSimulatedIndexerSensors() {
        return simulatedIndexerSensors;
    }

    public ClawSensorIOSim getSimulatedClawSensors() {
        return simulatedClawSensors;
    }

    public ElevatorSensorIOSim getSimulatedElevatorSensors() {
        return simulatedElevatorSensors;
    }

    public ClimberSensorIOSim getSimulatedClimberSensors() {
        return simulatedClimberSensors;
    }

    public SimTalonFXIO getSimulatedClawMotor() {
        return simulatedClawMotor;
    }

    public SimTalonFXIO getSimulateClimberPivot() {
        return simulatedClimberPivotMotor;
    }

    public SimTalonFXIO getSimulatedWristMotor() {
        return simulatedWristMotor;
    }

    public RobotViz getRobotViz() {
        return robotViz;
    }

    public LoggedDashboardChooser<DesiredMode> getModeChooser() {
        return autoModeSelector.getModeChooser();
    }

    public String getScoringSequence() {
        return autoModeSelector.getScoreOrder();
    }

    public String getLevelSequence() {
        return autoModeSelector.getLevelOrder();
    }

    public LoggedDashboardChooser<StartingPosition> getStartingPositionChooser() {
        return autoModeSelector.getStartingPositionChooser();
    }

    public LoggedDashboardChooser<Integer> getIceCreamCountChooser() {
        return autoModeSelector.getIceCreamCountChooser();
    }

    public LoggedDashboardChooser<FeederStrategy> getFeederStrategyChooser() {
        return autoModeSelector.getFeederStrategyChooser();
    }

    public boolean isValidAutoCommand() {
        return autoModeSelector.isValidCommand();
    }

    public SuperstructureStateMachine getStateMachine() {
        return stateMachine;
    }

    public boolean odometryCloseToPose(Pose2d pose) {
        Pose2d fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        double distance = fieldToRobot.getTranslation().getDistance(pose.getTranslation());
        SmartDashboard.putNumber("Distance From Start Pose", distance);
        double rotation =
                Math.abs(
                        fieldToRobot
                                .getRotation()
                                .rotateBy(pose.getRotation().unaryMinus())
                                .getDegrees());
        SmartDashboard.putNumber("Rotation From Start Pose", rotation);
        if (distance < 0.25 && rotation < 8.0) {
            return true;
        }
        return false;
    }

    public Command getAutonomousCommand() {
        return autoModeSelector.getAutonomousCommand();
    }

    public void setAutoDefaultCommands(FeederStrategy strategy) {
        this.getIntakePivot().setAutoDefaultCommand(strategy);
    }

    public Command getDisabledCommand() {
        return Commands.none();
    }

    public void setTeleopDefaultCommands() {
        this.getIntakePivot().setTeleopDefaultCommand();
        ;
    }
}
