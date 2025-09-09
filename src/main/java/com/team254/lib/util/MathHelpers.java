package com.team254.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MathHelpers {
    public static final Pose2d kPose2dZero = new Pose2d();

    public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
        return new Pose2d(kTranslation2dZero, rotation);
    }

    public static final Pose2d pose2dFromTranslation(Translation2d translation) {
        return new Pose2d(translation, kRotation2dZero);
    }

    public static final Rotation2d kRotation2dZero = new Rotation2d();
    public static final Rotation2d kRotation2dPi = Rotation2d.fromDegrees(180.0);

    public static final Translation2d kTranslation2dZero = new Translation2d();

    public static final Transform2d kTransform2dZero = new Transform2d();

    public static final Transform2d transform2dFromRotation(Rotation2d rotation) {
        return new Transform2d(kTranslation2dZero, rotation);
    }

    public static final Transform2d transform2dFromTranslation(Translation2d translation) {
        return new Transform2d(translation, kRotation2dZero);
    }

    public static double reverseInterpolate(
            Translation2d query, Translation2d start, Translation2d end) {
        Translation2d segment = end.minus(start);
        Translation2d queryToStart = query.minus(start);

        double segmentLengthSqr = segment.getX() * segment.getX() + segment.getY() * segment.getY();

        if (segmentLengthSqr == 0.0) { // start and end are the same point
            return 0.0;
        }

        return (queryToStart.getX() * segment.getX() + queryToStart.getY() * segment.getY())
                / segmentLengthSqr;
    }

    public static double distanceToLineSegment(
            Translation2d query, Translation2d start, Translation2d end) {
        double t = reverseInterpolate(query, start, end);
        if (t < 0.0) { // closest point is before start
            return query.getDistance(start);
        } else if (t > 1.0) { // closest point is after end
            return query.getDistance(end);
        } else { // closest point is within the segment
            Translation2d segment = end.minus(start);
            Translation2d closestPoint = start.plus(segment.times(t));
            return query.getDistance(closestPoint);
        }
    }

    public static double perpendicularDistanceToLine(
            Translation2d query, Translation2d start, Translation2d end) {
        double t = reverseInterpolate(query, start, end);
        Translation2d segment = end.minus(start);
        Translation2d closestPoint = start.plus(segment.times(t));
        return query.getDistance(closestPoint);
    }
}
