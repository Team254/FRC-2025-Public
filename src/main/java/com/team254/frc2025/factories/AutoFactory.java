package com.team254.frc2025.factories;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotContainer;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.commands.PathfindingAutoAlignCommand;
import com.team254.frc2025.subsystems.drive.DriveSubsystem;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.commands.ChezySequenceCommandGroup;
import com.team254.lib.pathplanner.controllers.PPHolonomicDriveController;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.reefscape.ReefBranch;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoFactory {

    public static Command getPathfindToFeederCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance) {
        return getPathfindToFeederCommand(
                driveSubsystem, robotState, letterSupplier, isRedAlliance, null);
    }

    public static Command getPathfindToFeederCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.FEEDER,
                        letterSupplier,
                        () -> 0.0,
                        () -> null,
                        () -> false,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToBargeCommand(
            RobotContainer container,
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            BooleanSupplier isRedAlliance,
            BooleanSupplier doneStagingSupplier,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.BARGE,
                        () -> false,
                        () -> 0.0,
                        () ->
                                container
                                        .getModalSuperstructureTriggers()
                                        .setFutureStateAndWaitUntilDesiredCommand(
                                                SuperstructureState.STAGE_BARGE, "BargeStage"),
                        doneStagingSupplier,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToFeederGroundCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.FEEDER_GROUND,
                        letterSupplier,
                        () -> 0.0,
                        () -> null,
                        () -> false,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToFeederGroundRetryCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.FEEDER_GROUND_RETRY,
                        letterSupplier,
                        () -> 0.0,
                        () -> null,
                        () -> false,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToAlgaeIntake(
            RobotContainer container,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            boolean shortJoinPath) {
        return getPathfindToAlgaeIntake(
                container, branchSupplier, letterSupplier, shortJoinPath, null);
    }

    public static Command getPathfindToAlgaeIntake(
            RobotContainer container,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            boolean shortJoinPath,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        return getPathfindToReefCommand(
                container.getDriveSubsystem(),
                container.getRobotState(),
                branchSupplier,
                letterSupplier,
                () -> 0.0,
                () ->
                        Commands.defer(
                                () ->
                                        container
                                                .getModalSuperstructureTriggers()
                                                .setFutureWaitDesiredReefIntakeAlgaeCommand(
                                                        branchSupplier.get()),
                                Set.of()),
                () ->
                        container.getStateMachine().getDesiredState()
                                == container
                                        .getModalSuperstructureTriggers()
                                        .getCorrectReefIntakeState(branchSupplier.get()),
                () -> container.getRobotState().isRedAlliance(),
                true,
                shortJoinPath,
                warmupCommands);
    }

    public static Command getPathfindToReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            Supplier<Double> heightSupplier,
            Supplier<Command> stageCommandSupplier,
            BooleanSupplier doneStagingSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            boolean shortJoinPath) {
        return getPathfindToReefCommand(
                driveSubsystem,
                robotState,
                branchSupplier,
                letterSupplier,
                heightSupplier,
                stageCommandSupplier,
                doneStagingSupplier,
                isRedAlliance,
                isAlgae,
                shortJoinPath,
                null);
    }

    public static Command getPathfindToReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            Supplier<Double> heightSupplier,
            Supplier<Command> stageCommandSupplier,
            BooleanSupplier doneStagingSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            boolean shortJoinPath,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        branchSupplier,
                        letterSupplier,
                        heightSupplier,
                        stageCommandSupplier,
                        doneStagingSupplier,
                        isRedAlliance,
                        isAlgae,
                        false,
                        shortJoinPath);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                        new InstantCommand(
                                () ->
                                        PPHolonomicDriveController.setControlPoint(
                                                Constants.DriveConstants.kDriveToCoralOffset)),
                        inner)
                .finallyDo(() -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero));
    }

    public static Command getPathfindBackoffFromReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae) {
        return getPathfindBackoffFromReefCommand(
                driveSubsystem,
                robotState,
                branchSupplier,
                letterSupplier,
                isRedAlliance,
                isAlgae,
                null);
    }

    public static Command getPathfindBackoffFromReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        branchSupplier,
                        letterSupplier,
                        () -> 0.0,
                        () -> null,
                        () -> false,
                        isRedAlliance,
                        isAlgae,
                        true,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                new InstantCommand(() -> Pathfinding.setBackoffObstacles()),
                inner,
                new InstantCommand(() -> Pathfinding.setTeleopObstacles()));
    }
}
