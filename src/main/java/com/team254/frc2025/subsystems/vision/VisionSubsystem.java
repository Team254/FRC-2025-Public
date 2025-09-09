package com.team254.frc2025.subsystems.vision;

import com.team254.frc2025.Constants;
import com.team254.frc2025.Constants.VisionConstants;
import com.team254.frc2025.RobotState;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem that processes AprilTag detections and provides robot pose estimates. Supports
 * multiple cameras and various pose estimation algorithms including MegaTag.
 */
public class VisionSubsystem extends SubsystemBase {

    private final VisionIO io;
    private final RobotState state;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    private boolean useVision = true;

    /** Creates a new vision subsystem. */
    public VisionSubsystem(VisionIO io, RobotState state) {
        this.io = io;
        this.state = state;
    }

    /** Fuses two vision pose estimates using inverse-variance weighting. */
    private VisionFieldPoseEstimate fuseEstimates(
            VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
        // Ensure b is the newer measurement
        if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
            VisionFieldPoseEstimate tmp = a;
            a = b;
            b = tmp;
        }

        // Preview both estimates to the same timestamp
        Transform2d a_T_b =
                state.getFieldToRobot(b.getTimestampSeconds())
                        .get()
                        .minus(state.getFieldToRobot(a.getTimestampSeconds()).get());

        Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
        Pose2d poseB = b.getVisionRobotPoseMeters();

        // Inverse‑variance weighting
        var varianceA =
                a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
        var varianceB =
                b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

        Rotation2d fusedHeading = poseB.getRotation();
        if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
                && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
            fusedHeading =
                    new Rotation2d(
                            poseA.getRotation().getCos() / varianceA.get(2, 0)
                                    + poseB.getRotation().getCos() / varianceB.get(2, 0),
                            poseA.getRotation().getSin() / varianceA.get(2, 0)
                                    + poseB.getRotation().getSin() / varianceB.get(2, 0));
        }

        double weightAx = 1.0 / varianceA.get(0, 0);
        double weightAy = 1.0 / varianceA.get(1, 0);
        double weightBx = 1.0 / varianceB.get(0, 0);
        double weightBy = 1.0 / varianceB.get(1, 0);

        Pose2d fusedPose =
                new Pose2d(
                        new Translation2d(
                                (poseA.getTranslation().getX() * weightAx
                                                + poseB.getTranslation().getX() * weightBx)
                                        / (weightAx + weightBx),
                                (poseA.getTranslation().getY() * weightAy
                                                + poseB.getTranslation().getY() * weightBy)
                                        / (weightAy + weightBy)),
                        fusedHeading);

        Matrix<N3, N1> fusedStdDev =
                VecBuilder.fill(
                        Math.sqrt(1.0 / (weightAx + weightBx)),
                        Math.sqrt(1.0 / (weightAy + weightBy)),
                        Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

        int numTags = a.getNumTags() + b.getNumTags();
        double time = b.getTimestampSeconds();

        return new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags);
    }

    @Override
    public void periodic() {
        double startTime = RobotTime.getTimestampSeconds();
        io.readInputs(inputs);

        logCameraInputs("Vision/CameraA", inputs.cameraA);
        logCameraInputs("Vision/CameraB", inputs.cameraB);

        var maybeMTA = processCamera(inputs.cameraA, "CameraA", VisionConstants.kRobotToCameraA);
        var maybeMTB = processCamera(inputs.cameraB, "CameraB", VisionConstants.kRobotToCameraB);

        if (!useVision) {
            Logger.recordOutput("Vision/usingVision", false);
            Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
            Logger.recordOutput(
                    "Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - startTime);
            return;
        }

        Logger.recordOutput("Vision/usingVision", true);

        Optional<VisionFieldPoseEstimate> accepted = Optional.empty();
        if (maybeMTA.isPresent() != maybeMTB.isPresent()) {
            accepted = maybeMTA.isPresent() ? maybeMTA : maybeMTB;
        } else if (maybeMTA.isPresent() && maybeMTB.isPresent()) {
            accepted = Optional.of(fuseEstimates(maybeMTA.get(), maybeMTB.get()));
        }

        accepted.ifPresent(
                est -> {
                    Logger.recordOutput("Vision/fusedAccepted", est.getVisionRobotPoseMeters());
                    state.updateMegatagEstimate(est);
                });

        Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
        Logger.recordOutput(
                "Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - startTime);
    }

    private void logCameraInputs(String prefix, VisionIO.VisionIOInputs.CameraInputs cam) {
        Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
        Logger.recordOutput(prefix + "/MegatagCount", cam.megatagCount);

        if (DriverStation.isDisabled()) {
            SmartDashboard.putBoolean(prefix + "/SeesTarget", cam.seesTarget);
            SmartDashboard.putNumber(prefix + "/MegatagCount", cam.megatagCount);
        }

        if (cam.pose3d != null) {
            Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
        }

        if (cam.megatagPoseEstimate != null) {
            Logger.recordOutput(
                    prefix + "/MegatagPoseEstimate", cam.megatagPoseEstimate.fieldToRobot());
            Logger.recordOutput(prefix + "/Quality", cam.megatagPoseEstimate.quality());
            Logger.recordOutput(prefix + "/AvgTagArea", cam.megatagPoseEstimate.avgTagArea());
        }

        if (cam.fiducialObservations != null) {
            Logger.recordOutput(prefix + "/FiducialCount", cam.fiducialObservations.length);
        }
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.VisionIOInputs.CameraInputs cam, String label, Transform2d robotToCamera) {

        String logPrefix = "Vision/" + label;

        if (!cam.seesTarget) {
            return Optional.empty();
        }

        Optional<VisionFieldPoseEstimate> estimate = Optional.empty();

        if (cam.megatagPoseEstimate != null) {
            Optional<VisionFieldPoseEstimate> mtEstimate =
                    processMegatagPoseEstimate(cam.megatagPoseEstimate, cam, logPrefix);

            mtEstimate.ifPresent(
                    est ->
                            Logger.recordOutput(
                                    logPrefix + "/AcceptedMegatagEstimate",
                                    est.getVisionRobotPoseMeters()));

            Optional<VisionFieldPoseEstimate> gyroEstimate =
                    fuseWithGyro(cam.megatagPoseEstimate, cam, logPrefix);

            gyroEstimate.ifPresent(
                    est ->
                            Logger.recordOutput(
                                    logPrefix + "/FuseWithGyroEstimate",
                                    est.getVisionRobotPoseMeters()));

            // Prefer Megatag when available
            if (mtEstimate.isPresent()) {
                estimate = mtEstimate;
                Logger.recordOutput(logPrefix + "/AcceptMegatag", true);
                Logger.recordOutput(logPrefix + "/AcceptGyro", false);
            } else if (gyroEstimate.isPresent()) {
                estimate = gyroEstimate;
                Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
                Logger.recordOutput(logPrefix + "/AcceptGyro", true);
            } else {
                Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
                Logger.recordOutput(logPrefix + "/AcceptGyro", false);
            }
        }

        return estimate;
    }

    private Optional<VisionFieldPoseEstimate> fuseWithGyro(
            MegatagPoseEstimate poseEstimate,
            VisionIO.VisionIOInputs.CameraInputs cam,
            String logPrefix) {

        if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
            return Optional.empty();
        }

        // Use Megatag directly when 2 or more tags are visible
        if (poseEstimate.fiducialIds().length > 1) {
            return Optional.empty();
        }

        // Reject if the robot is yawing rapidly (time‑sync unreliable)
        final double kHighYawLookbackS = 0.3;
        final double kHighYawVelocityRadS = 5.0;

        if (state.getMaxAbsDriveYawAngularVelocityInRange(
                                poseEstimate.timestampSeconds() - kHighYawLookbackS,
                                poseEstimate.timestampSeconds())
                        .orElse(Double.POSITIVE_INFINITY)
                > kHighYawVelocityRadS) {
            return Optional.empty();
        }

        var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
        if (priorPose.isEmpty()) {
            return Optional.empty();
        }

        var maybeFieldToTag =
                Constants.kAprilTagLayoutReefsOnly.getTagPose(poseEstimate.fiducialIds()[0]);
        if (maybeFieldToTag.isEmpty()) {
            return Optional.empty();
        }

        Pose2d fieldToTag =
                new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);

        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());

        Pose2d posteriorPose =
                new Pose2d(
                        fieldToTag
                                .getTranslation()
                                .minus(
                                        robotToTag
                                                .getTranslation()
                                                .rotateBy(priorPose.get().getRotation())),
                        priorPose.get().getRotation());

        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
        double xyStd = Math.max(xStd, yStd);

        return Optional.of(
                new VisionFieldPoseEstimate(
                        posteriorPose,
                        poseEstimate.timestampSeconds(),
                        VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
                        poseEstimate.fiducialIds().length));
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
            MegatagPoseEstimate poseEstimate,
            VisionIO.VisionIOInputs.CameraInputs cam,
            String logPrefix) {

        if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
            return Optional.empty();
        }

        // Single‑tag extra checks
        if (poseEstimate.fiducialIds().length < 2) {
            for (var fiducial : cam.fiducialObservations) {
                if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
                    return Optional.empty();
                }
            }

            if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
                return Optional.empty();
            }

            var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
            if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
                    && priorPose.isPresent()) {
                double yawDiff =
                        Math.abs(
                                MathUtil.angleModulus(
                                        priorPose.get().getRotation().getRadians()
                                                - poseEstimate
                                                        .fieldToRobot()
                                                        .getRotation()
                                                        .getRadians()));

                if (yawDiff > Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)) {
                    return Optional.empty();
                }
            }
        }

        if (poseEstimate.fieldToRobot().getTranslation().getNorm()
                < VisionConstants.kDefaultNormThreshold) {
            return Optional.empty();
        }

        if (Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
            return Optional.empty();
        }

        // Exclusive‑tag filtering
        var exclusiveTag = state.getExclusiveTag();
        boolean hasExclusiveId =
                exclusiveTag.isPresent()
                        && java.util.Arrays.stream(poseEstimate.fiducialIds())
                                .anyMatch(id -> id == exclusiveTag.get());

        if (exclusiveTag.isPresent() && !hasExclusiveId) {
            return Optional.empty();
        }

        var loggedPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
        if (loggedPose.isEmpty()) {
            return Optional.empty();
        }

        Pose2d estimatePose = poseEstimate.fieldToRobot();

        double scaleFactor = 1.0 / poseEstimate.quality();
        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;
        double rotStd =
                cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex] * scaleFactor;

        double xyStd = Math.max(xStd, yStd);
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

        return Optional.of(
                new VisionFieldPoseEstimate(
                        estimatePose,
                        poseEstimate.timestampSeconds(),
                        visionStdDevs,
                        poseEstimate.fiducialIds().length));
    }

    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
    }
}
