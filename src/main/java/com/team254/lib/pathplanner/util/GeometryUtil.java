package com.team254.lib.pathplanner.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

/** Utility class for various geometry functions used during generation */
public class GeometryUtil {
    /**
     * Quadratic interpolation between Translation2ds
     *
     * @param a Position 1
     * @param b Position 2
     * @param c Position 3
     * @param t Interpolation factor (0.0-1.0)
     * @return Interpolated value
     */
    public static Translation2d quadraticLerp(
            Translation2d a, Translation2d b, Translation2d c, double t) {
        Translation2d p0 = a.interpolate(b, t);
        Translation2d p1 = b.interpolate(c, t);
        return p0.interpolate(p1, t);
    }

    /**
     * Cubic interpolation between Translation2ds
     *
     * @param a Position 1
     * @param b Position 2
     * @param c Position 3
     * @param d Position 4
     * @param t Interpolation factor (0.0-1.0)
     * @return Interpolated value
     */
    public static Translation2d cubicLerp(
            Translation2d a, Translation2d b, Translation2d c, Translation2d d, double t) {
        Translation2d p0 = quadraticLerp(a, b, c, t);
        Translation2d p1 = quadraticLerp(b, c, d, t);
        return p0.interpolate(p1, t);
    }

    public static double signedInstantaneousCurvature(
            Translation2d p1, Translation2d p2, Translation2d p3, Translation2d p4, double t) {
        // Calculate the first derivative (velocity)
        Translation2d v =
                p1.minus(p2)
                        .times(3 * (1 - t) * (1 - t))
                        .plus(p2.minus(p3).times(6 * (1 - t) * t))
                        .plus(p3.minus(p4).times(3 * t * t));

        // Calculate the second derivative (acceleration)
        Translation2d a =
                p1.minus(p2)
                        .times(6 * (1 - t))
                        .plus(p2.minus(p3).times(6 * (t - 1)))
                        .plus(p3.minus(p4).times(6 * t));

        // Calculate the curvature
        double numerator = v.getX() * a.getY() - v.getY() * a.getX();
        double v_norm_sqr = v.getX() * v.getX() + v.getY() * v.getY();
        double denominator = v_norm_sqr * Math.sqrt(v_norm_sqr);

        if (denominator == 0) {
            return Double.POSITIVE_INFINITY; // Handle the case for a straight line
        }

        return numerator / denominator; // This is the curvature
    }

    /**
     * Calculate the curve radius given 3 points on the curve
     *
     * @param a Point A
     * @param b Point B
     * @param c Point C
     * @return Curve radius
     */
    public static double calculateRadius(Translation2d a, Translation2d b, Translation2d c) {
        Translation2d vba = a.minus(b);
        Translation2d vbc = c.minus(b);
        double cross_z = (vba.getX() * vbc.getY()) - (vba.getY() * vbc.getX());
        double dotP = (vba.getX() * vbc.getX()) + (vba.getY() * vbc.getY());
        int sign = (cross_z < 0) ? 1 : -1;
        if (Math.abs(cross_z) < 0.1 && dotP >= 0) {
            return sign * 0.01;
        }

        double ab = a.getDistance(b);
        double bc = b.getDistance(c);
        double ac = a.getDistance(c);

        double p = (ab + bc + ac) / 2;
        double area = Math.sqrt(Math.abs(p * (p - ab) * (p - bc) * (p - ac)));
        return sign * (ab * bc * ac) / (4 * area);
    }

    public static Optional<Translation2d> intersectionPoint(
            Translation2d vectorStart,
            Rotation2d vectorRotation,
            Translation2d segmentStart,
            Translation2d segmentEnd) {
        // 1. Represent the Line and Line Segment:
        // Line: P = vectorStart + t * rotatedVector
        Translation2d rotatedVector =
                new Translation2d(1, 0).rotateBy(vectorRotation); // Direction vector
        // Line Segment: S = segmentStart + u * (segmentEnd - segmentStart)
        Translation2d segmentVector = segmentEnd.minus(segmentStart);

        // 2. Finding the Intersection Point (of the Lines):
        // vectorStart.x + t * rotatedVector.x = segmentStart.x + u * segmentVector.x
        // vectorStart.y + t * rotatedVector.y = segmentStart.y + u * segmentVector.y

        double a = rotatedVector.getX();
        double b = -segmentVector.getX();
        double c = rotatedVector.getY();
        double d = -segmentVector.getY();
        double e = segmentStart.getX() - vectorStart.getX();
        double f = segmentStart.getY() - vectorStart.getY();

        double det = a * d - b * c;

        if (Math.abs(det) < 1e-9) { // Lines are parallel (or nearly parallel)
            return Optional.empty(); // No intersection
        }

        double t = (e * d - b * f) / det;

        // Calculate intersection point
        double intersectionX = vectorStart.getX() + t * rotatedVector.getX();
        double intersectionY = vectorStart.getY() + t * rotatedVector.getY();

        return Optional.of(new Translation2d(intersectionX, intersectionY));
    }
}
