package com.team254.frc2025.subsystems.vision;

import com.team254.frc2025.Constants;
import com.team254.frc2025.Constants.VisionConstants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.simulation.SimulatedRobotState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Simulation implementation of VisionIO using PhotonVision simulation. Extends Limelight hardware
 * implementation to reuse data processing logic.
 */
public class VisionIOSimPhoton extends VisionIOHardwareLimelight {
    private final PhotonCamera camera = new PhotonCamera("camera");
    private final PhotonCamera cameraB = new PhotonCamera("cameraB");
    private PhotonCameraSim cameraSim;
    private PhotonCameraSim cameraBSim;
    private final VisionSystemSim visionSim;
    private final SimulatedRobotState simRobotState;

    private final int kResWidth = 1280;
    private final int kResHeight = 800;

    /** Creates a new simulated vision IO instance using PhotonVision. */
    public VisionIOSimPhoton(RobotState state, SimulatedRobotState simRobotState) {
        super(state);
        this.simRobotState = simRobotState;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(Constants.kAprilTagLayoutReefsOnly);

        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
        prop.setCalibError(0.35, 0.5);
        prop.setFPS(45);
        prop.setAvgLatencyMs(20);
        prop.setLatencyStdDevMs(5);
        prop.setExposureTimeMs(0.65);

        cameraSim = new PhotonCameraSim(camera, prop);
        cameraSim.setMinTargetAreaPixels(1000);
        cameraBSim = new PhotonCameraSim(cameraB, prop);
        cameraBSim.setMinTargetAreaPixels(1000);

        Transform3d robotToCameraA =
                new Transform3d(
                        new Translation3d(
                                VisionConstants.kRobotToCameraAForward,
                                VisionConstants.kRobotToCameraASide,
                                VisionConstants.kCameraAHeightOffGroundMeters),
                        new Rotation3d(
                                0.0, // Roll
                                -VisionConstants.kCameraAPitchRads, // Pitch
                                VisionConstants.kCameraAYawOffset.getRadians() // Yaw
                                ));

        Transform3d robotToCameraB =
                new Transform3d(
                        new Translation3d(
                                VisionConstants.kRobotToCameraBForward,
                                VisionConstants.kRobotToCameraBSide,
                                VisionConstants.kCameraBHeightOffGroundMeters),
                        new Rotation3d(
                                0.0, // Roll
                                -VisionConstants.kCameraBPitchRads, // Pitch
                                VisionConstants.kCameraBYawOffset.getRadians() // Yaw
                                ));

        visionSim.addCamera(cameraSim, robotToCameraA);
        visionSim.addCamera(cameraBSim, robotToCameraB);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);
        cameraBSim.enableRawStream(true);
        cameraBSim.enableProcessedStream(true);
        cameraBSim.enableDrawWireframe(true);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        Pose2d estimatedPose = simRobotState.getLatestFieldToRobot();
        if (estimatedPose != null) {
            visionSim.update(estimatedPose);
            Logger.recordOutput("Vision/SimIO/updateSimPose", estimatedPose);
        }

        NetworkTable table =
                NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightATableName);
        NetworkTable tableB =
                NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);

        writeToTable(camera.getAllUnreadResults(), table, cameraSim);
        writeToTable(cameraB.getAllUnreadResults(), tableB, cameraBSim);

        super.readInputs(inputs);
    }

    /** Generates robot pose data from PhotonVision results. */
    private List<Double> getBotpose(
            Transform3d fieldToCamera,
            int numTags,
            PhotonPipelineResult result,
            PhotonCameraSim cameraSim) {
        if (result == null || result.targets.isEmpty()) return null;

        Optional<Transform3d> optRobotToCamera =
                visionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());
        Pose3d fieldToRobot;
        if (optRobotToCamera.isPresent()) {
            Transform3d cameraToRobot = optRobotToCamera.get().inverse();
            Pose3d robotPose3d =
                    new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                            .transformBy(cameraToRobot);
            fieldToRobot = robotPose3d;
        } else {
            fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
        }

        List<Double> pose_data =
                new ArrayList<>(
                        Arrays.asList(
                                fieldToRobot.getX(),
                                fieldToRobot.getY(),
                                fieldToRobot.getZ(),
                                0.0,
                                0.0,
                                fieldToRobot.getRotation().getMeasureZ().in(Units.Degree),
                                result.metadata.getLatencyMillis(),
                                (double) numTags,
                                0.0,
                                0.0,
                                result.getBestTarget().getArea()));

        for (var target : result.targets) {
            pose_data.addAll(
                    Arrays.asList(
                            (double) target.getFiducialId(),
                            target.getYaw(), // txnc
                            target.getPitch(), // tync
                            target.area, // ta
                            0.0, // distToCamera
                            0.0, // distToRobot
                            target.getPoseAmbiguity() // ambiguity
                            ));
        }
        return pose_data;
    }

    /**
     * Writes simulated vision data to NetworkTables for consumption by Limelight processing code.
     */
    private void writeToTable(
            List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
        boolean seesTarget = false;
        for (var result : results) {
            List<Double> pose_data = null;
            if (result.getMultiTagResult().isPresent()) {
                var multiTagResult = result.getMultiTagResult().get();
                Transform3d best = multiTagResult.estimatedPose.best;

                pose_data =
                        getBotpose(best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);
            } else if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();
                Transform3d best =
                        Constants.kAprilTagLayoutReefsOnly
                                .getTagPose(bestTarget.getFiducialId())
                                .get()
                                .minus(Pose3d.kZero)
                                .plus(bestTarget.bestCameraToTarget.inverse());

                pose_data = getBotpose(best, 1, result, cameraSim);
            }

            if (pose_data != null) {
                table.getEntry("botpose_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                table.getEntry("botpose_orb_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
                // MT2pitch,
                // MT2yaw]
                table.getEntry("stddevs")
                        .setDoubleArray(
                                new Double[] {
                                    0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                });
                seesTarget = true;
            }
            table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
        }
        table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
    }
}
