package com.team254.lib.reefscape;

import com.team254.frc2025.Constants;
import com.team254.lib.util.FieldConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ScoringLocation {

    private final Pose2d pose;
    private final int scoringLocationIndex;

    public ScoringLocation(Pose2d pose, int scoringLocationIndex) {
        this.pose = pose;
        this.scoringLocationIndex = scoringLocationIndex;
    }

    public Pose2d getPose() {
        return pose;
    }

    public int getScoringLocationIndex() {
        return scoringLocationIndex;
    }

    @Override
    public String toString() {
        return "ScoringLocation: " + scoringLocationIndex + " " + pose;
    }

    public String getName() {
        return String.valueOf((char) ('A' + scoringLocationIndex));
    }

    public Pose2d getPoseWithoutReefBuffer() {
        double offset = Constants.kAutoAlignReefUnbuffer;
        Translation2d offsetTranslation = new Translation2d(offset, 0);
        return pose.plus(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    public Pose2d getPosePlusTransform(double offset) {
        Translation2d offsetTranslation = new Translation2d(offset, 0);
        return pose.plus(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    public Pose2d getPoseWithoutReefBufferPlusTransform(double offset) {
        Translation2d offsetTranslation = new Translation2d(offset, 0);
        return getPoseWithoutReefBuffer()
                .plus(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    public static ScoringLocation getLocation(String loc) {
        switch (loc.toUpperCase()) {
            case "A":
                return A;
            case "B":
                return B;
            case "C":
                return C;
            case "D":
                return D;
            case "E":
                return E;
            case "F":
                return F;
            case "G":
                return G;
            case "H":
                return H;
            case "I":
                return I;
            case "J":
                return J;
            case "K":
                return K;
            case "L":
                return L;
            case "AB":
                return AB;
            case "CD":
                return CD;
            case "EF":
                return EF;
            case "GH":
                return GH;
            case "IJ":
                return IJ;
            case "KL":
                return KL;
            default:
                return FeederLeft;
        }
    }

    public static ScoringLocation getLocation(ReefBranch branch, boolean useFirstLetter) {
        if (branch == ReefBranch.FEEDER
                || branch == ReefBranch.FEEDER_GROUND
                || branch == ReefBranch.FEEDER_GROUND_RETRY) {
            return (useFirstLetter) ? FeederLeft : FeederRight;
        } else if (branch == ReefBranch.BARGE) {
            return Barge;
        } else {
            char letter = useFirstLetter ? branch.getLeftLetter() : branch.getRightLetter();
            return getLocation(Character.toString(letter));
        }
    }

    public static ScoringLocation getLocationAlgae(ReefBranch branch) {
        if (branch == ReefBranch.FEEDER || branch == ReefBranch.FEEDER_GROUND) {
            throw new IllegalArgumentException("Algae location not applicable for feeder");
        } else {
            return getLocation(branch.getLeftLetter() + "" + branch.getRightLetter());
        }
    }

    public static boolean isAlgaeReefIntakeLocation(ScoringLocation location) {
        return location == AB
                || location == CD
                || location == EF
                || location == GH
                || location == IJ
                || location == KL;
    }

    // Should Only be used in auto for reef branches or feeders
    public static Pair<ReefBranch, Boolean> getSelectors(ScoringLocation location) {
        if (location == FeederLeft || location == FeederRight) {
            return new Pair<>(ReefBranch.FEEDER, location == FeederLeft);
        } else if (location == AB) {
            return new Pair<>(ReefBranch.AB, false);
        } else if (location == CD) {
            return new Pair<>(ReefBranch.CD, false);
        } else if (location == EF) {
            return new Pair<>(ReefBranch.EF, false);
        } else if (location == GH) {
            return new Pair<>(ReefBranch.GH, false);
        } else if (location == IJ) {
            return new Pair<>(ReefBranch.IJ, false);
        } else if (location == KL) {
            return new Pair<>(ReefBranch.KL, false);
        } else {
            switch (location.getScoringLocationIndex()) {
                case 0:
                    return new Pair<>(ReefBranch.AB, true);
                case 1:
                    return new Pair<>(ReefBranch.AB, false);
                case 2:
                    return new Pair<>(ReefBranch.CD, true);
                case 3:
                    return new Pair<>(ReefBranch.CD, false);
                case 4:
                    return new Pair<>(ReefBranch.EF, true);
                case 5:
                    return new Pair<>(ReefBranch.EF, false);
                case 6:
                    return new Pair<>(ReefBranch.GH, true);
                case 7:
                    return new Pair<>(ReefBranch.GH, false);
                case 8:
                    return new Pair<>(ReefBranch.IJ, true);
                case 9:
                    return new Pair<>(ReefBranch.IJ, false);
                case 10:
                    return new Pair<>(ReefBranch.KL, true);
                case 11:
                    return new Pair<>(ReefBranch.KL, false);
                default:
                    throw new IllegalArgumentException(
                            "Invalid scoring location index: "
                                    + location.getScoringLocationIndex());
            }
        }
    }

    public static final ScoringLocation A = new ScoringLocation(Constants.kBranchPoses[1], 0);
    public static final ScoringLocation B = new ScoringLocation(Constants.kBranchPoses[0], 1);
    public static final ScoringLocation C = new ScoringLocation(Constants.kBranchPoses[11], 2);
    public static final ScoringLocation D = new ScoringLocation(Constants.kBranchPoses[10], 3);
    public static final ScoringLocation E = new ScoringLocation(Constants.kBranchPoses[9], 4);
    public static final ScoringLocation F = new ScoringLocation(Constants.kBranchPoses[8], 5);
    public static final ScoringLocation G = new ScoringLocation(Constants.kBranchPoses[7], 6);
    public static final ScoringLocation H = new ScoringLocation(Constants.kBranchPoses[6], 7);
    public static final ScoringLocation I = new ScoringLocation(Constants.kBranchPoses[5], 8);
    public static final ScoringLocation J = new ScoringLocation(Constants.kBranchPoses[4], 9);
    public static final ScoringLocation K = new ScoringLocation(Constants.kBranchPoses[3], 10);
    public static final ScoringLocation L = new ScoringLocation(Constants.kBranchPoses[2], 11);

    public static final ScoringLocation AB = new ScoringLocation(Constants.kAlgaePoses[0], -1);
    public static final ScoringLocation CD = new ScoringLocation(Constants.kAlgaePoses[5], -1);
    public static final ScoringLocation EF = new ScoringLocation(Constants.kAlgaePoses[4], -1);
    public static final ScoringLocation GH = new ScoringLocation(Constants.kAlgaePoses[3], -1);
    public static final ScoringLocation IJ = new ScoringLocation(Constants.kAlgaePoses[2], -1);
    public static final ScoringLocation KL = new ScoringLocation(Constants.kAlgaePoses[1], -1);

    public static final ScoringLocation FeederRight =
            new ScoringLocation(Constants.kFeederRightPose, -1);
    public static final ScoringLocation FeederLeft =
            new ScoringLocation(Constants.kFeederLeftPose, -1);

    public static final ScoringLocation Barge = new ScoringLocation(Constants.kBargePose, -1);

    public static final ScoringLocation IceCreamLeft =
            new ScoringLocation(FieldConstants.StagingPositions.leftIceCream, -1);
    public static final ScoringLocation IceCreamMiddle =
            new ScoringLocation(FieldConstants.StagingPositions.middleIceCream, -1);
    public static final ScoringLocation IceCreamRight =
            new ScoringLocation(FieldConstants.StagingPositions.rightIceCream, -1);
}
