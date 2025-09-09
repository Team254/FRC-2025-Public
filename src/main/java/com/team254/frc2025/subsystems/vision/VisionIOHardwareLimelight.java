package com.team254.frc2025.subsystems.vision;

import com.team254.frc2025.Constants;
import com.team254.frc2025.Constants.VisionConstants;
import com.team254.frc2025.RobotState;
import com.team254.lib.limelight.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicReference;

/** Hardware implementation of VisionIO using Limelight cameras. */
public class VisionIOHardwareLimelight implements VisionIO {
    NetworkTable tableA =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightATableName);
    NetworkTable tableB =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);
    RobotState robotState;
    AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());
    int imuMode = 1;

    private static final double[] DEFAULT_STDDEVS =
            new double[VisionConstants.kExpectedStdDevArrayLength];

    /** Creates a new Limelight vision IO instance. */
    public VisionIOHardwareLimelight(RobotState robotState) {
        this.robotState = robotState;
        setLLSettings();
    }

    /** Configures Limelight camera poses in robot coordinate system. */
    private void setLLSettings() {
        double[] cameraAPose = {
            Constants.VisionConstants.kRobotToCameraAForward,
            Constants.VisionConstants.kRobotToCameraASide,
            VisionConstants.kCameraAHeightOffGroundMeters,
            0.0,
            VisionConstants.kCameraAPitchDegrees,
            VisionConstants.kCameraAYawOffset.getDegrees()
        };

        tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);

        double[] cameraBPose = {
            Constants.VisionConstants.kRobotToCameraBForward,
            Constants.VisionConstants.kRobotToCameraBSide,
            VisionConstants.kCameraBHeightOffGroundMeters,
            0.0,
            VisionConstants.kCameraBPitchDegrees,
            VisionConstants.kCameraBYawOffset.getDegrees()
        };

        tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBPose);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightATableName);
        readCameraData(tableB, inputs.cameraB, VisionConstants.kLimelightBTableName);
        latestInputs.set(inputs);
    }

    /** Reads data from a single Limelight camera. */
    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        if (camera.seesTarget) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var robotPose3d =
                        LimelightHelpers.toPose3D(
                                LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (megatag != null) {
                    camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatagCount = megatag.tagCount;
                    camera.fiducialObservations =
                            FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (robotPose3d != null) {
                    camera.pose3d = robotPose3d;
                }

                camera.standardDeviations =
                        table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);
            } catch (Exception e) {
                System.err.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }
}
