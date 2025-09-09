package com.team254.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

/** Interface for vision system hardware abstraction. */
public interface VisionIO {

    /** Container for all vision input data. */
    class VisionIOInputs {
        /** Input data from a single camera. */
        public static class CameraInputs {
            public boolean seesTarget;
            public FiducialObservation[] fiducialObservations;
            public MegatagPoseEstimate megatagPoseEstimate;
            public MegatagPoseEstimate megatag2PoseEstimate;
            public int megatag2Count;
            public int megatagCount;
            public Pose3d pose3d;
            public double[] standardDeviations =
                    new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
            // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        }

        public CameraInputs cameraA = new CameraInputs();
        public CameraInputs cameraB = new CameraInputs();
    }

    void readInputs(VisionIOInputs inputs);
}
