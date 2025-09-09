package com.team254.frc2025.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.frc2025.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.lib.util.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code DriveIO} interface defines the input/output operations for the drivetrain. It provides
 * methods to control and monitor the swerve drive system.
 */
public interface DriveIO {

    @AutoLog
    class DriveIOInputs extends SwerveDriveState {
        public double gyroAngle = 0.0;

        DriveIOInputs() {
            this.Pose = MathHelpers.kPose2dZero;
        }

        public void fromSwerveDriveState(SwerveDriveState stateIn) {
            this.Pose = stateIn.Pose;
            this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
            this.FailedDaqs = stateIn.FailedDaqs;
            this.ModuleStates = stateIn.ModuleStates;
            this.ModuleTargets = stateIn.ModuleTargets;
            this.Speeds = stateIn.Speeds;
            this.OdometryPeriod = stateIn.OdometryPeriod;
        }
    }

    void readInputs(DriveIOInputs inputs);

    void logModules(SwerveDriveState driveState);

    void resetOdometry(Pose2d pose);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate);

    void setStateStdDevs(double xStd, double yStd, double rotStd);
}
