package com.team254.frc2025.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.frc2025.Constants;
import com.team254.frc2025.Robot;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.drive.DriveSubsystem;
import com.team254.frc2025.subsystems.superstructure.CoralStateTracker.CoralPosition;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.util.FieldConstants;
import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveMaintainingHeadingCommand extends Command {
    public DriveMaintainingHeadingCommand(
            DriveSubsystem drivetrain,
            RobotState robotState,
            RobotContainer robotContainer,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier turn) {
        mDrivetrain = drivetrain;
        mRobotState = robotState;
        mRobotContainer = robotContainer;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

        driveWithHeading.HeadingController.setPID(
                Constants.DriveConstants.kHeadingControllerP,
                Constants.DriveConstants.kHeadingControllerI,
                Constants.DriveConstants.kHeadingControllerD);

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    private final RobotState mRobotState;
    private final RobotContainer mRobotContainer;
    protected DriveSubsystem mDrivetrain;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mTurnSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private final SwerveRequest.FieldCentric driveNoHeading =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Constants.DriveConstants.kDriveMaxSpeed
                                    * 0.05) // Add a 5% deadband in open loop
                    .withRotationalDeadband(
                            Constants.DriveConstants.kDriveMaxAngularRate
                                    * Constants.kSteerJoystickDeadband)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(Constants.DriveConstants.kDriveMaxSpeed * 0.05)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * Constants.DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * Constants.DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double throttleFieldFrame = mRobotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = mRobotState.isRedAlliance() ? -strafe : strafe;
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband
                || (Util.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                        && Math.abs(
                                        mRobotState.getLatestRobotRelativeChassisSpeed()
                                                .omegaRadiansPerSecond)
                                > Math.toRadians(10))) {
            mDrivetrain.setControl(
                    (driveNoHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withRotationalRate(
                                    turnFieldFrame
                                            * Constants.DriveConstants.kDriveMaxAngularRate)));
            mHeadingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint =
                        Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());
            }
            Logger.recordOutput("DriveMaintainHeading/throttleFieldFrame", throttleFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/strafeFieldFrame", strafeFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/mHeadingSetpoint", mHeadingSetpoint.get());
            if (mRobotContainer.getCoralStateTracker().getCurrentPosition() != CoralPosition.NONE
                    && mRobotContainer.getModalControls().coralMode().getAsBoolean()
                    && mRobotContainer
                            .getModalSuperstructureTriggers()
                            .getEnableSnapHeadingCoral()
                            .get()) {
                double targetAngle =
                        mRobotContainer
                                .getStateMachine()
                                .getReefFaceAngleRadians(
                                        mRobotContainer
                                                .getStateMachine()
                                                .getClosestFace(mRobotState.isRedAlliance()));
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(
                                        mRobotState.isRedAlliance()
                                                ? Util.flipRedBlue(new Rotation2d(targetAngle))
                                                : new Rotation2d(targetAngle)));
                Logger.recordOutput("DriveMaintainHeading/reefHeadingLock", targetAngle);
                // update heading setpoint to avoid snapping back to previous setpoint after scoring
                mHeadingSetpoint =
                        Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());

                // barge heading lock
            } else if (mRobotContainer.getModalControls().algaeClimbMode().getAsBoolean()
                    && mRobotContainer
                                    .getModalSuperstructureTriggers()
                                    .getLatestAlgaeStageState()
                                    .get()
                            == SuperstructureState.STAGE_BARGE
                    && mRobotContainer
                            .getModalSuperstructureTriggers()
                            .getEnableSnapHeadingAlgae()
                            .get()) {
                double targetAngle =
                        mRobotState.getLatestFieldToRobot().getValue().getX()
                                        > FieldConstants.fieldLength / 2.0
                                ? 0
                                : Math.PI;
                if (Math.abs(
                                mRobotState.getLatestFieldToRobot().getValue().getX()
                                        - FieldConstants.fieldLength / 2.0)
                        > 1.0) {
                    mHeadingSetpoint = Optional.of(new Rotation2d(targetAngle));
                }
                Logger.recordOutput("DriveMaintainHeading/bargeHeadingLock", targetAngle);
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));

                // processor heading lock
            } else if (mRobotContainer.getModalControls().algaeClimbMode().getAsBoolean()
                    && mRobotContainer
                                    .getModalSuperstructureTriggers()
                                    .getLatestAlgaeStageState()
                                    .get()
                            == SuperstructureState.STAGE_PROCESSOR
                    && mRobotContainer
                            .getModalSuperstructureTriggers()
                            .getEnableSnapHeadingAlgae()
                            .get()) {
                double targetAngle =
                        mRobotContainer.getRobotState().isRedAlliance()
                                ? Math.PI / 2
                                : -Math.PI / 2;
                if (mRobotContainer.getRobotState().onOpponentSide()) {
                    targetAngle += Math.PI;
                }
                mHeadingSetpoint = Optional.of(new Rotation2d(targetAngle));
                Logger.recordOutput("DriveMaintainHeading/processorHeadingLock", targetAngle);
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));

                // normal maintain heading
            } else {
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));
            }
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput(
                    "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
