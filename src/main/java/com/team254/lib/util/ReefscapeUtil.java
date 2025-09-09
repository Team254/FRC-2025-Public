package com.team254.lib.util;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class ReefscapeUtil {
    public static boolean awayFromReef(RobotContainer container) {
        return awayFromReef(container, 0.6);
    }

    public static boolean awayFromReef(RobotContainer container, double offset) {

        Translation2d blueReefCenter = FieldConstants.Reef.center;
        Translation2d redReefCenter = Util.flipRedBlue(blueReefCenter);

        Pose2d robotPose = container.getRobotState().getLatestFieldToRobot().getValue();
        Pose2d predictedRobotPose = container.getRobotState().getPredictedCappedFieldToRobot(0.2);
        Transform2d offsetTransform =
                new Transform2d(new Translation2d(offset, 0), Rotation2d.kZero);
        Pose2d robotPoseOffset = predictedRobotPose.plus(offsetTransform);

        double blueDistance = robotPoseOffset.getTranslation().getDistance(blueReefCenter);
        double redDistance = robotPoseOffset.getTranslation().getDistance(redReefCenter);
        boolean positionCheck =
                blueDistance > Constants.kReefRadius && redDistance > Constants.kReefRadius;
        Logger.recordOutput("ReefscapeUtils/positionCheck", positionCheck);

        double originalDistanceAwayFromBlueReef =
                robotPose.getTranslation().getDistance(blueReefCenter);
        double futureDistanceAwayFromBlueReef =
                predictedRobotPose.getTranslation().getDistance(blueReefCenter);

        double originalDistanceAwayFromRedReef =
                robotPose.getTranslation().getDistance(redReefCenter);
        double futureDistanceAwayFromRedReef =
                predictedRobotPose.getTranslation().getDistance(redReefCenter);

        boolean predictedCheck =
                (originalDistanceAwayFromBlueReef <= futureDistanceAwayFromBlueReef
                                || Util.epsilonEquals(
                                        originalDistanceAwayFromBlueReef,
                                        futureDistanceAwayFromBlueReef,
                                        0.02))
                        && (originalDistanceAwayFromRedReef <= futureDistanceAwayFromRedReef
                                || Util.epsilonEquals(
                                        originalDistanceAwayFromRedReef,
                                        futureDistanceAwayFromRedReef,
                                        0.02));

        Logger.recordOutput("ReefscapeUtils/predictedCheck", predictedCheck);

        boolean gyroCheck = container.getElevator().gyroCheck();
        Logger.recordOutput("ReefscapeUtils/gyroCheck", gyroCheck);

        return positionCheck && gyroCheck;
    }
}
