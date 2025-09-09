package com.team254.frc2025.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.drive.DriveSubsystem;
import com.team254.lib.util.MathHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class AutoAlignToPoseCommand extends Command {
    private ProfiledPIDController driveController;
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            Constants.DriveConstants.kDriveMaxAngularRate,
                            Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                    0.02);
    private DriveSubsystem driveSubsystem;
    private RobotState robotState;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.0, ffMaxRadius = 0.1;
    private Pose2d targetLocation;

    public AutoAlignToPoseCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Pose2d targetLocation,
            double constraintFactor) {
        this.driveSubsystem = driveSubsystem;
        this.targetLocation = targetLocation;
        this.robotState = robotState;
        this.driveController =
                new ProfiledPIDController(
                        Constants.AutoConstants.kPXYController,
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                Constants.DriveConstants.kDriveMaxSpeed * constraintFactor,
                                Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                                        * constraintFactor),
                        0.02);
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // arm center is the same as the robot center when stowed, so can use field to
        // robot
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();

        driveController.reset(
                currentPose.getTranslation().getDistance(targetLocation.getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(
                                        robotState.getLatestMeasuredFieldRelativeChassisSpeeds()
                                                .vxMetersPerSecond,
                                        robotState.getLatestMeasuredFieldRelativeChassisSpeeds()
                                                .vyMetersPerSecond)
                                .rotateBy(
                                        targetLocation
                                                .getTranslation()
                                                .minus(
                                                        robotState
                                                                .getLatestFieldToRobot()
                                                                .getValue()
                                                                .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(),
                robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(2.0));

        driveController.setTolerance(0.04);
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();

        Logger.recordOutput("DriveToPose/currentPose", currentPose);
        Logger.recordOutput("DriveToPose/targetLocation", targetLocation.toString());
        Logger.recordOutput("DriveToPose/targetPose", targetLocation);

        double currentDistance =
                currentPose.getTranslation().getDistance(targetLocation.getTranslation());
        double ffScaler =
                MathUtil.clamp(
                        (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveErrorAbs = currentDistance;
        Logger.recordOutput("DriveToPose/ffScaler", ffScaler);
        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScaler
                        + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

        // Calculate theta speed
        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScaler
                        + thetaController.calculate(
                                currentPose.getRotation().getRadians(),
                                targetLocation.getRotation().getRadians());
        thetaErrorAbs =
                Math.abs(
                        currentPose.getRotation().minus(targetLocation.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity =
                MathHelpers.pose2dFromRotation(
                                currentPose
                                        .getTranslation()
                                        .minus(targetLocation.getTranslation())
                                        .getAngle())
                        .transformBy(
                                MathHelpers.transform2dFromTranslation(
                                        new Translation2d(driveVelocityScalar, 0.0)))
                        .getTranslation();
        driveSubsystem.setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        driveVelocity.getX(),
                                        driveVelocity.getY(),
                                        thetaVelocity,
                                        currentPose.getRotation())));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setControl(new SwerveRequest.ApplyRobotSpeeds());
    }

    @Override
    public boolean isFinished() {
        return targetLocation.equals(null)
                || (driveController.atGoal() && thetaController.atGoal());
    }
}
