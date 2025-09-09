package com.team254.frc2025.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.simulation.SimulatedRobotState;
import com.team254.frc2025.utils.simulations.MapleSimSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveIOSim} class extends {@link DriveIOHardware} to provide simulation-specific
 * functionality for the swerve drive system. It integrates with WPILib's simulation framework for
 * testing and development.
 */
public class DriveIOSim extends DriveIOHardware {

    private SimulatedRobotState simRobotState = null;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;
    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    Pose2d lastConsumedPose = null;
    Consumer<SwerveDriveState> simTelemetryConsumer =
            swerveDriveState -> {
                // Protect at init
                if (simRobotState == null) {
                    return;
                }

                if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
                    swerveDriveState.Pose =
                            mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                }
                simRobotState.addFieldToRobot(swerveDriveState.Pose);
                telemetryConsumer_.accept(swerveDriveState);
            };

    public DriveIOSim(
            RobotState robotState,
            SimulatedRobotState simRobotState,
            SwerveDrivetrainConstants driveTrainConstants,
            @SuppressWarnings("rawtypes") SwerveModuleConstants... modules) {
        super(robotState, driveTrainConstants, modules);
        this.simRobotState = simRobotState;

        // Rewrite the telemetry consumer with a consumer for sim
        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (Constants.useMapleSim) {
            mapleSimSwerveDrivetrain =
                    new MapleSimSwerveDrivetrain(
                            Units.Seconds.of(kSimLoopPeriod),
                            Units.Pounds.of(Constants.DriveConstants.kRobotWeightPounds),
                            Units.Inches.of(Constants.DriveConstants.kBumperWidthInches),
                            Units.Inches.of(Constants.DriveConstants.kBumperLengthInches),
                            DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                            DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                            1.2,
                            getModuleLocations(),
                            getPigeon2(),
                            getModules(),
                            SimTunerConstants.FrontLeft,
                            SimTunerConstants.FrontRight,
                            SimTunerConstants.BackLeft,
                            SimTunerConstants.BackRight);
            simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        } else {
            lastSimTime = Utils.getCurrentTimeSeconds();
            simNotifier =
                    new Notifier(
                            () -> {
                                final double currentTime = Utils.getCurrentTimeSeconds();
                                double deltaTime = currentTime - lastSimTime;
                                lastSimTime = currentTime;
                                updateSimState(deltaTime, RobotController.getBatteryVoltage());
                            });
        }
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetOdometry(Pose2d pose) {
        if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetOdometry(pose);
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        super.readInputs(inputs);

        // Handle the viz
        var pose = simRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", simRobotState.getLatestFieldToRobot());
        }
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return mapleSimSwerveDrivetrain;
    }
}
