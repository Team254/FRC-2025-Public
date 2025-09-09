package com.team254.frc2025.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveIOHardware} class implements the {@link DriveIO} interface using CTRE's
 * SwerveDrivetrain with TalonFX motors and CANcoders. It provides hardware-level control for the
 * swerve drive system.
 */
public class DriveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements DriveIO {

    AtomicReference<SwerveDriveState> telemetryCache_ = new AtomicReference<>();

    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;

    private RobotState robotState_;

    public DriveIOHardware(
            RobotState robotState,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, 250.0, modules);
        this.robotState_ = robotState;

        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        roll = getPigeon2().getRoll();
        pitch = getPigeon2().getPitch();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();

        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                angularPitchVelocity,
                angularRollVelocity,
                roll,
                pitch,
                accelerationX,
                accelerationY);

        this.getOdometryThread().setThreadPriority(99);

        registerTelemetry(telemetryConsumer_);
    }

    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
            this.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()));
        } else {
            this.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()),
                    visionFieldPoseEstimate.getVisionMeasurementStdDevs());
        }
    }

    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
        this.setStateStdDevs(stateStdDevs);
    }

    Consumer<SwerveDriveState> telemetryConsumer_ =
            swerveDriveState -> {
                telemetryCache_.set(swerveDriveState.clone());
                robotState_.addOdometryMeasurement(
                        (RobotTime.getTimestampSeconds() - Utils.getCurrentTimeSeconds())
                                + swerveDriveState.Timestamp,
                        swerveDriveState.Pose);
            };

    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        if (telemetryCache_.get() == null) return;
        inputs.fromSwerveDriveState(telemetryCache_.get());
        var gyroRotation = inputs.Pose.getRotation();
        inputs.gyroAngle = gyroRotation.getDegrees();
        var measuredRobotRelativeChassisSpeeds =
                getKinematics().toChassisSpeeds(inputs.ModuleStates);
        var measuredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        measuredRobotRelativeChassisSpeeds, gyroRotation);
        var desiredRobotRelativeChassisSpeeds =
                getKinematics().toChassisSpeeds(inputs.ModuleTargets);
        var desiredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        desiredRobotRelativeChassisSpeeds, gyroRotation);

        BaseStatusSignal.refreshAll(
                angularRollVelocity,
                angularPitchVelocity,
                angularYawVelocity,
                pitch,
                roll,
                accelerationX,
                accelerationY);

        double timestamp = RobotTime.getTimestampSeconds();
        double rollRadsPerS = Units.degreesToRadians(angularRollVelocity.getValueAsDouble());
        double pitchRadsPerS = Units.degreesToRadians(angularPitchVelocity.getValueAsDouble());
        double yawRadsPerS = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        // Trust gyro rate more than odometry.
        var fusedFieldRelativeChassisSpeeds =
                new ChassisSpeeds(
                        measuredFieldRelativeChassisSpeeds.vxMetersPerSecond,
                        measuredFieldRelativeChassisSpeeds.vyMetersPerSecond,
                        yawRadsPerS);

        double pitchRads = Units.degreesToRadians(pitch.getValueAsDouble());
        double rollRads = Units.degreesToRadians(roll.getValueAsDouble());
        double accelX = accelerationX.getValueAsDouble();
        double accelY = accelerationY.getValueAsDouble();
        robotState_.addDriveMotionMeasurements(
                timestamp,
                rollRadsPerS,
                pitchRadsPerS,
                yawRadsPerS,
                pitchRads,
                rollRads,
                accelX,
                accelY,
                desiredRobotRelativeChassisSpeeds,
                desiredFieldRelativeChassisSpeeds,
                measuredRobotRelativeChassisSpeeds,
                measuredFieldRelativeChassisSpeeds,
                fusedFieldRelativeChassisSpeeds);
    }

    @Override
    public void logModules(SwerveDriveState driveState) {
        final String[] moduleNames = {"Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR"};
        if (driveState.ModuleStates == null) return;
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(
                    moduleNames[i] + " Absolute Encoder Angle",
                    getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(
                    moduleNames[i] + " Steering Angle", driveState.ModuleStates[i].angle);
            Logger.recordOutput(
                    moduleNames[i] + " Target Steering Angle", driveState.ModuleTargets[i].angle);
            Logger.recordOutput(
                    moduleNames[i] + " Drive Velocity",
                    driveState.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(
                    moduleNames[i] + " Target Drive Velocity",
                    driveState.ModuleTargets[i].speedMetersPerSecond);
        }
    }
}
