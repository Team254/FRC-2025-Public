package com.team254.frc2025.auto;

import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.reefscape.ScoringLocation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AutoModeSelector {
    public enum DesiredMode {
        CUSTOM,
        DO_NOTHING,
    }

    public enum FeederStrategy {
        FUNNEL,
        GROUND
    }

    public enum StartingPosition {
        LEFT_BARGE("Left Side", new Pose2d(7, 7.0, Rotation2d.fromDegrees(225.0))),
        MIDDLE_BARGE("Middle Side", new Pose2d(7.2, 4, Rotation2d.fromDegrees(180.0))),
        RIGHT_BARGE("Right Side", new Pose2d(7, 1.0, Rotation2d.fromDegrees(135.0)));

        private final String displayName;
        private final Pose2d startingPose;

        StartingPosition(String displayName, Pose2d startingPose) {
            this.startingPose = startingPose;
            this.displayName = displayName;
        }

        public Pose2d getStartingPose() {
            return startingPose;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    public enum ScoringPosition {
        A(ScoringLocation.A, SuperstructureState.STAGE_CORAL_L4, "Position A"),
        B(ScoringLocation.B, SuperstructureState.STAGE_CORAL_L4, "Position B"),
        C(ScoringLocation.C, SuperstructureState.STAGE_CORAL_L4, "Position C"),
        D(ScoringLocation.D, SuperstructureState.STAGE_CORAL_L4, "Position D"),
        E(ScoringLocation.E, SuperstructureState.STAGE_CORAL_L4, "Position E"),
        F(ScoringLocation.F, SuperstructureState.STAGE_CORAL_L4, "Position F"),
        G(ScoringLocation.G, SuperstructureState.STAGE_CORAL_L4, "Position G"),
        H(ScoringLocation.H, SuperstructureState.STAGE_CORAL_L4, "Position H"),
        I(ScoringLocation.I, SuperstructureState.STAGE_CORAL_L4, "Position I"),
        J(ScoringLocation.J, SuperstructureState.STAGE_CORAL_L4, "Position J"),
        K(ScoringLocation.K, SuperstructureState.STAGE_CORAL_L4, "Position K"),
        L(ScoringLocation.L, SuperstructureState.STAGE_CORAL_L4, "Position L"),

        A_L3(ScoringLocation.A, SuperstructureState.STAGE_CORAL_L3, "Position A(L3)"),
        B_L3(ScoringLocation.B, SuperstructureState.STAGE_CORAL_L3, "Position B(L3)"),
        C_L3(ScoringLocation.C, SuperstructureState.STAGE_CORAL_L3, "Position C(L3)"),
        D_L3(ScoringLocation.D, SuperstructureState.STAGE_CORAL_L3, "Position D(L3)"),
        E_L3(ScoringLocation.E, SuperstructureState.STAGE_CORAL_L3, "Position E(L3)"),
        F_L3(ScoringLocation.F, SuperstructureState.STAGE_CORAL_L3, "Position F(L3)"),
        G_L3(ScoringLocation.G, SuperstructureState.STAGE_CORAL_L3, "Position G(L3)"),
        H_L3(ScoringLocation.H, SuperstructureState.STAGE_CORAL_L3, "Position H(L3)"),
        I_L3(ScoringLocation.I, SuperstructureState.STAGE_CORAL_L3, "Position I(L3)"),
        J_L3(ScoringLocation.J, SuperstructureState.STAGE_CORAL_L3, "Position J(L3)"),
        K_L3(ScoringLocation.K, SuperstructureState.STAGE_CORAL_L3, "Position K(L3)"),
        L_L3(ScoringLocation.L, SuperstructureState.STAGE_CORAL_L3, "Position L(L3)"),

        A_L2(ScoringLocation.A, SuperstructureState.STAGE_CORAL_L2, "Position A(L2)"),
        B_L2(ScoringLocation.B, SuperstructureState.STAGE_CORAL_L2, "Position B(L2)"),
        C_L2(ScoringLocation.C, SuperstructureState.STAGE_CORAL_L2, "Position C(L2)"),
        D_L2(ScoringLocation.D, SuperstructureState.STAGE_CORAL_L2, "Position D(L2)"),
        E_L2(ScoringLocation.E, SuperstructureState.STAGE_CORAL_L2, "Position E(L2)"),
        F_L2(ScoringLocation.F, SuperstructureState.STAGE_CORAL_L2, "Position F(L2)"),
        G_L2(ScoringLocation.G, SuperstructureState.STAGE_CORAL_L2, "Position G(L2)"),
        H_L2(ScoringLocation.H, SuperstructureState.STAGE_CORAL_L2, "Position H(L2)"),
        I_L2(ScoringLocation.I, SuperstructureState.STAGE_CORAL_L2, "Position I(L2)"),
        J_L2(ScoringLocation.J, SuperstructureState.STAGE_CORAL_L2, "Position J(L2)"),
        K_L2(ScoringLocation.K, SuperstructureState.STAGE_CORAL_L2, "Position K(L2)"),
        L_L2(ScoringLocation.L, SuperstructureState.STAGE_CORAL_L2, "Position L(L2)"),

        AB_ALGAE(ScoringLocation.AB, SuperstructureState.REEF_ALGAE_INTAKE_L3, "Position AB"),
        CD_ALGAE(ScoringLocation.CD, SuperstructureState.REEF_ALGAE_INTAKE_L2, "Position CD"),
        EF_ALGAE(ScoringLocation.EF, SuperstructureState.REEF_ALGAE_INTAKE_L3, "Position EF"),
        GH_ALGAE(ScoringLocation.GH, SuperstructureState.REEF_ALGAE_INTAKE_L2, "Position GH"),
        IJ_ALGAE(ScoringLocation.IJ, SuperstructureState.REEF_ALGAE_INTAKE_L3, "Position IJ"),
        KL_ALGAE(ScoringLocation.KL, SuperstructureState.REEF_ALGAE_INTAKE_L2, "Position KL");

        private final ScoringLocation scoringLocation;
        private final SuperstructureState state;
        private final String displayName;

        ScoringPosition(
                ScoringLocation scoringLocation, SuperstructureState state, String displayName) {
            this.scoringLocation = scoringLocation;
            this.state = state;
            this.displayName = displayName;
        }

        public ScoringLocation getScoringLocation() {
            return scoringLocation;
        }

        public SuperstructureState getState() {
            return state;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    private final RobotContainer container;
    private final LoggedDashboardChooser<StartingPosition> startPositionChooser =
            new LoggedDashboardChooser<>("Starting Position");
    private final LoggedDashboardChooser<DesiredMode> modeChooser =
            new LoggedDashboardChooser<>("Auto Mode");
    private final LoggedDashboardChooser<Integer> iceCreamChooser =
            new LoggedDashboardChooser<>("Ice Cream Count");
    private final LoggedDashboardChooser<FeederStrategy> feederStrategyChooser =
            new LoggedDashboardChooser<>("Feeder Strategy");
    private boolean validCommand = true;
    private final LoggedNetworkString scoreOrder =
            new LoggedNetworkString("/SmartDashboard/scoreOrder", "IKLLKL");
    private final LoggedNetworkString levelOrder =
            new LoggedNetworkString("/SmartDashboard/levelOrder", "44433*");

    public AutoModeSelector(RobotContainer container) {
        this.container = container;

        // Add auto mode options with more descriptive names
        modeChooser.addDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        modeChooser.addOption("Custom Auto", DesiredMode.CUSTOM);

        startPositionChooser.addDefaultOption(
                StartingPosition.LEFT_BARGE.getDisplayName(), StartingPosition.LEFT_BARGE);
        startPositionChooser.addOption(
                StartingPosition.MIDDLE_BARGE.getDisplayName(), StartingPosition.MIDDLE_BARGE);
        startPositionChooser.addOption(
                StartingPosition.RIGHT_BARGE.getDisplayName(), StartingPosition.RIGHT_BARGE);

        iceCreamChooser.addDefaultOption("Zero", 0);
        iceCreamChooser.addOption("One", 1);
        iceCreamChooser.addOption("Two", 2);
        iceCreamChooser.addOption("Three", 3);

        feederStrategyChooser.addDefaultOption("Ground", FeederStrategy.GROUND);
        feederStrategyChooser.addOption("Funnel", FeederStrategy.FUNNEL);
    }

    public LoggedDashboardChooser<DesiredMode> getModeChooser() {
        return modeChooser;
    }

    public static ScoringPosition getScoringPosition(char letter, char level) {
        return switch (level) {
            case '4' -> getL4Position(letter);
            case '3' -> getL3Position(letter);
            case '2' -> getL2Position(letter);
            case '*' -> getReefAlgaePosition(letter);
            default -> throw new IllegalArgumentException("Invalid level letter: " + level);
        };
    }

    private static ScoringPosition getReefAlgaePosition(char letter) {
        switch (letter) {
            case 'a':
                return ScoringPosition.AB_ALGAE;
            case 'b':
                return ScoringPosition.AB_ALGAE;
            case 'c':
                return ScoringPosition.CD_ALGAE;
            case 'd':
                return ScoringPosition.CD_ALGAE;
            case 'e':
                return ScoringPosition.EF_ALGAE;
            case 'f':
                return ScoringPosition.EF_ALGAE;
            case 'g':
                return ScoringPosition.GH_ALGAE;
            case 'h':
                return ScoringPosition.GH_ALGAE;
            case 'i':
                return ScoringPosition.IJ_ALGAE;
            case 'j':
                return ScoringPosition.IJ_ALGAE;
            case 'k':
                return ScoringPosition.KL_ALGAE;
            case 'l':
                return ScoringPosition.KL_ALGAE;
            default:
                throw new IllegalArgumentException("Invalid scoring position letter: " + letter);
        }
    }

    private static ScoringPosition getL2Position(char letter) {
        switch (letter) {
            case 'a':
                return ScoringPosition.A_L2;
            case 'b':
                return ScoringPosition.B_L2;
            case 'c':
                return ScoringPosition.C_L2;
            case 'd':
                return ScoringPosition.D_L2;
            case 'e':
                return ScoringPosition.E_L2;
            case 'f':
                return ScoringPosition.F_L2;
            case 'g':
                return ScoringPosition.G_L2;
            case 'h':
                return ScoringPosition.H_L2;
            case 'i':
                return ScoringPosition.I_L2;
            case 'j':
                return ScoringPosition.J_L2;
            case 'k':
                return ScoringPosition.K_L2;
            case 'l':
                return ScoringPosition.L_L2;
            default:
                throw new IllegalArgumentException("Invalid scoring position letter: " + letter);
        }
    }

    private static ScoringPosition getL3Position(char letter) {
        switch (letter) {
            case 'a':
                return ScoringPosition.A_L3;
            case 'b':
                return ScoringPosition.B_L3;
            case 'c':
                return ScoringPosition.C_L3;
            case 'd':
                return ScoringPosition.D_L3;
            case 'e':
                return ScoringPosition.E_L3;
            case 'f':
                return ScoringPosition.F_L3;
            case 'g':
                return ScoringPosition.G_L3;
            case 'h':
                return ScoringPosition.H_L3;
            case 'i':
                return ScoringPosition.I_L3;
            case 'j':
                return ScoringPosition.J_L3;
            case 'k':
                return ScoringPosition.K_L3;
            case 'l':
                return ScoringPosition.L_L3;
            default:
                throw new IllegalArgumentException("Invalid scoring position letter: " + letter);
        }
    }

    private static ScoringPosition getL4Position(char letter) {
        switch (letter) {
            case 'a':
                return ScoringPosition.A;
            case 'b':
                return ScoringPosition.B;
            case 'c':
                return ScoringPosition.C;
            case 'd':
                return ScoringPosition.D;
            case 'e':
                return ScoringPosition.E;
            case 'f':
                return ScoringPosition.F;
            case 'g':
                return ScoringPosition.G;
            case 'h':
                return ScoringPosition.H;
            case 'i':
                return ScoringPosition.I;
            case 'j':
                return ScoringPosition.J;
            case 'k':
                return ScoringPosition.K;
            case 'l':
                return ScoringPosition.L;
            default:
                throw new IllegalArgumentException("Invalid scoring position letter: " + letter);
        }
    }

    public String getScoreOrder() {
        return scoreOrder.get();
    }

    public String getLevelOrder() {
        return levelOrder.get();
    }

    public LoggedDashboardChooser<StartingPosition> getStartingPositionChooser() {
        return startPositionChooser;
    }

    public LoggedDashboardChooser<Integer> getIceCreamCountChooser() {
        return iceCreamChooser;
    }

    public LoggedDashboardChooser<FeederStrategy> getFeederStrategyChooser() {
        return feederStrategyChooser;
    }

    public Command getAutonomousCommand() {
        DesiredMode selectedMode = modeChooser.get();
        if (selectedMode == null) {
            return Commands.none();
        }
        switch (selectedMode) {
            case DO_NOTHING:
                validCommand = true;
                return this.container
                        .getModalSuperstructureTriggers()
                        .setStateAndWaitCommand(
                                SuperstructureState.STOW_CORAL, "Stow for do nothing");
            case CUSTOM:
                // Handle custom auto sequences
                StartingPosition startPos = startPositionChooser.get();
                Integer iceCreamCnt = iceCreamChooser.get();
                FeederStrategy feederStrategy = feederStrategyChooser.get();
                String scoringSequence = getScoreOrder();
                String levelSequence = getLevelOrder();
                if (startPos == null
                        || scoringSequence.equals("")
                        || iceCreamCnt == null
                        || feederStrategy == null) {
                    validCommand = false;
                    return Commands.none();
                }
                validCommand = true;

                return new PathfindingAuto(
                        container,
                        scoringSequence,
                        levelSequence,
                        startPos,
                        iceCreamCnt,
                        feederStrategy,
                        () -> container.getRobotState().isRedAlliance());
            default:
                return Commands.none();
        }
    }

    public boolean isValidCommand() {
        return validCommand;
    }
}
