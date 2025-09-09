package com.team254.frc2025.viz;

import com.team254.lib.util.FieldConstants;
import edu.wpi.first.math.geometry.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

// Custom class to visualize the reef.  Used in simulation and for log replay.
// Forked with changes from MapleSim.ReefscapeReefSimulation
public class ReefViz {

    interface CoralHolder {
        /**
         * Displays the positions of the CORALs that are on this holder.
         *
         * @param coralPosesToDisplay a list of poses used to visualize CORALs on AdvantageScope
         */
        void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay);
    }

    static final class Branch implements CoralHolder {
        private final Pose3d idealCoralPlacementPose;
        public boolean hasCoral;

        Branch(
                Translation2d idealPlacementPosition,
                Rotation2d facingOutwards,
                double heightMeters,
                double branchInwardsDirectionPitchRad) {
            this.idealCoralPlacementPose =
                    new Pose3d(
                            idealPlacementPosition.getX(),
                            idealPlacementPosition.getY(),
                            heightMeters,
                            new Rotation3d(
                                    0,
                                    -branchInwardsDirectionPitchRad,
                                    facingOutwards.plus(Rotation2d.k180deg).getRadians()));
            this.hasCoral = false;
        }

        @Override
        public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
            if (hasCoral) coralPosesToDisplay.add(idealCoralPlacementPose);
        }
    }

    final class Trough implements CoralHolder {
        private final Pose3d firstPlacementPose, secondPlacementPose;
        public int coralCount;

        protected Trough(Translation2d centerPosition, Rotation2d outwardsFacing) {
            Rotation3d coralRotation =
                    new Rotation3d(0, 0, outwardsFacing.plus(Rotation2d.kCCW_90deg).getRadians());
            Translation2d firstPosition =
                    centerPosition.plus(new Translation2d(0.08, outwardsFacing));
            Translation2d secondPosition =
                    centerPosition.plus(new Translation2d(-0.04, outwardsFacing));
            this.firstPlacementPose =
                    new Pose3d(firstPosition.getX(), firstPosition.getY(), 0.48, coralRotation);
            this.secondPlacementPose =
                    new Pose3d(secondPosition.getX(), secondPosition.getY(), 0.52, coralRotation);
            this.coralCount = 0;
        }

        @Override
        public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
            if (coralCount > 0) coralPosesToDisplay.add(firstPlacementPose);
            if (coralCount > 1) coralPosesToDisplay.add(secondPlacementPose);
        }
    }

    class Tower {
        public final Trough L1;
        public final Branch L2, L3, L4;

        public Tower(Translation2d stickCenterPositionOnField, Rotation2d facingOutwards) {
            // L1 trough, 15cm away from center
            this.L1 =
                    new Trough(
                            stickCenterPositionOnField.plus(
                                    new Translation2d(0.15, facingOutwards)),
                            facingOutwards);

            // L2 stick, 20 cm away from center, 78cm above ground, 35 deg pitch
            this.L2 =
                    new Branch(
                            stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
                            facingOutwards,
                            0.77,
                            Math.toRadians(-35));

            // L3 stick, 20 cm away from center, 118cm above ground, 35 deg pitch
            this.L3 =
                    new Branch(
                            stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
                            facingOutwards,
                            1.17,
                            Math.toRadians(-35));

            // L4 stick, 30 cm away from center, 178cm above ground, vertical
            this.L4 =
                    new Branch(
                            stickCenterPositionOnField.plus(
                                    new Translation2d(0.26, facingOutwards)),
                            facingOutwards,
                            1.78,
                            Math.toRadians(-90));
        }

        /** Clears all the CORALs on the BRANCHES */
        public void clearBranches() {
            L1.coralCount = 0;
            L2.hasCoral = L3.hasCoral = L4.hasCoral = false;
        }

        /** Obtains a collection of the {@link CoralHolder}s that are in this BRANCHES tower. */
        public Collection<CoralHolder> coralHolders() {
            return List.of(L1, L2, L3, L4);
        }
    }

    private final List<Tower> blueTowers;
    private final List<Tower> redTowers;
    private final List<CoralHolder> coralHolders;

    private static final ReefViz instance = new ReefViz();

    public static ReefViz getInstance() {
        return instance;
    }

    private ReefViz() {
        this.coralHolders = new ArrayList<>(96);
        this.blueTowers = new ArrayList<>(12);
        this.redTowers = new ArrayList<>(12);

        Translation2d origin =
                new Translation2d(
                        FieldMirroringUtils.FIELD_WIDTH / 2, FieldMirroringUtils.FIELD_HEIGHT / 2);
        Translation2d[] branchesCenterPositionBlue =
                new Translation2d[] {
                    new Translation2d(-4.810, 0.164).plus(origin), // A
                    new Translation2d(-4.810, -0.164).plus(origin), // B
                    new Translation2d(-4.690, -0.373).plus(origin), // C
                    new Translation2d(-4.406, -0.538).plus(origin), // D
                    new Translation2d(-4.164, -0.537).plus(origin), // E
                    new Translation2d(-3.879, -0.374).plus(origin), // F
                    new Translation2d(-3.759, -0.164).plus(origin), // G
                    new Translation2d(-3.759, 0.164).plus(origin), // H
                    new Translation2d(-3.880, 0.373).plus(origin), // I
                    new Translation2d(-4.164, 0.538).plus(origin), // J
                    new Translation2d(-4.405, 0.538).plus(origin), // K
                    new Translation2d(-4.690, 0.374).plus(origin) // L
                };
        Translation2d[] branchesCenterPositionRed =
                Arrays.stream(branchesCenterPositionBlue)
                        .map(FieldMirroringUtils::flip)
                        .toArray(Translation2d[]::new);
        Rotation2d[] branchesFacingOutwardsBlue =
                new Rotation2d[] {
                    Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
                    Rotation2d.fromDegrees(-120), Rotation2d.fromDegrees(-120), // C and D
                    Rotation2d.fromDegrees(-60), Rotation2d.fromDegrees(-60), // E and F
                    Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
                    Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
                    Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
                };
        Rotation2d[] branchesFacingOutwardsRed =
                Arrays.stream(branchesFacingOutwardsBlue)
                        .map(FieldMirroringUtils::flip)
                        .toArray(Rotation2d[]::new);

        Tower branch;
        for (int i = 0; i < 12; i++) {
            // blue
            branch = new Tower(branchesCenterPositionBlue[i], branchesFacingOutwardsBlue[i]);
            blueTowers.add(branch);
            coralHolders.addAll(branch.coralHolders());

            // red
            branch = new Tower(branchesCenterPositionRed[i], branchesFacingOutwardsRed[i]);
            redTowers.add(branch);
            coralHolders.addAll(branch.coralHolders());
        }
    }

    public void updateViz() {
        List<Pose3d> coralPieces = new ArrayList<>();
        for (var coralHolder : coralHolders) {
            coralHolder.addContainedCoralsForDisplay(coralPieces);
        }
        Logger.recordOutput("ReefViz/Coral", coralPieces.toArray(Pose3d[]::new));
    }

    public void addCoral(
            FieldConstants.ReefHeight reefHeight, FieldConstants.BranchCode code, boolean isRed) {
        List<Tower> selectedSide = this.blueTowers;
        if (isRed) selectedSide = this.redTowers;

        if (code.indexOffset > selectedSide.size()) {
            System.out.println("Index offset too large in ReefViz");
            return;
        }

        Tower selectedTower = selectedSide.get(code.indexOffset);
        switch (reefHeight) {
            case L1 -> selectedTower.L1.coralCount = selectedTower.L1.coralCount + 1;
            case L2 -> selectedTower.L2.hasCoral = true;
            case L3 -> selectedTower.L3.hasCoral = true;
            case L4 -> selectedTower.L4.hasCoral = true;
        }
    }
}
