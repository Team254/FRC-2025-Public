package com.team254.lib.pathplanner.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team254.lib.pathplanner.commands.FollowPathCommand;
import com.team254.lib.pathplanner.commands.PathPlannerAuto;
import com.team254.lib.pathplanner.commands.PathfindingCommand;
import com.team254.lib.pathplanner.config.RobotConfig;
import com.team254.lib.pathplanner.path.IPathCallback;
import com.team254.lib.pathplanner.path.PathConstraints;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/** Utility class used to build auto routines */
public class AutoBuilder {
    private static boolean configured = false;

    private static Supplier<Pose2d> poseSupplier;
    private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
    private static Consumer<Pose2d> resetPose;
    private static BooleanSupplier shouldFlipPath;
    private static boolean isHolonomic;

    // Pathfinding builders
    private static boolean pathfindingConfigured = false;
    private static TriFunction<Pose2d, PathConstraints, Double, Command>
            pathfindToPoseCommandBuilder;
    private static TriFunction<PathPlannerPath, PathConstraints, IPathCallback, Command>
            pathfindThenFollowPathCommandBuilder;

    /**
     * Configures the AutoBuilder for using PathPlanner's built-in commands.
     *
     * @param poseSupplier a supplier for the robot's current pose
     * @param resetPose a consumer for resetting the robot's pose
     * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
     *     speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow paths
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side
     *     of the field. This will maintain a global blue alliance origin.
     * @param driveRequirements the subsystem requirements for the robot's drive train
     */
    public static void configure(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Subsystem... driveRequirements) {
        if (configured) {
            DriverStation.reportError(
                    "Auto builder has already been configured. This is likely in error.", true);
        }

        AutoBuilder.pathFollowingCommandBuilder =
                (path) ->
                        new FollowPathCommand(
                                path,
                                poseSupplier,
                                robotRelativeSpeedsSupplier,
                                controller,
                                robotConfig,
                                shouldFlipPath,
                                driveRequirements);
        AutoBuilder.poseSupplier = poseSupplier;
        AutoBuilder.resetPose = resetPose;
        AutoBuilder.configured = true;
        AutoBuilder.shouldFlipPath = shouldFlipPath;
        AutoBuilder.isHolonomic = robotConfig.isHolonomic;

        AutoBuilder.pathfindToPoseCommandBuilder =
                (pose, constraints, goalEndVel) ->
                        new PathfindingCommand(
                                pose,
                                constraints,
                                goalEndVel,
                                poseSupplier,
                                robotRelativeSpeedsSupplier,
                                controller,
                                robotConfig,
                                driveRequirements);
        AutoBuilder.pathfindThenFollowPathCommandBuilder =
                (targetPath, constraints, pathCallback) ->
                        new PathfindingCommand(
                                targetPath,
                                pathCallback,
                                constraints,
                                poseSupplier,
                                robotRelativeSpeedsSupplier,
                                controller,
                                robotConfig,
                                shouldFlipPath,
                                driveRequirements);
        AutoBuilder.pathfindingConfigured = true;
    }

    /**
     * Configures the AutoBuilder with custom path following command builder. Building pathfinding
     * commands is not supported if using a custom command builder. Custom path following commands
     * will not have the path flipped for them, and event markers will not be triggered
     * automatically.
     *
     * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
     * @param poseSupplier a supplier for the robot's current pose
     * @param resetPose a consumer for resetting the robot's pose
     * @param shouldFlipPose Supplier that determines if the starting pose should be flipped to the
     *     other side of the field. This will maintain a global blue alliance origin. NOTE: paths
     *     will not be flipped when configured with a custom path following command. Flipping the
     *     paths must be handled in your command.
     * @param isHolonomic Does the robot have a holonomic drivetrain
     */
    public static void configureCustom(
            Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            BooleanSupplier shouldFlipPose,
            boolean isHolonomic) {
        if (configured) {
            DriverStation.reportError(
                    "Auto builder has already been configured. This is likely in error.", true);
        }

        AutoBuilder.pathFollowingCommandBuilder = pathFollowingCommandBuilder;
        AutoBuilder.poseSupplier = poseSupplier;
        AutoBuilder.resetPose = resetPose;
        AutoBuilder.configured = true;
        AutoBuilder.shouldFlipPath = shouldFlipPose;
        AutoBuilder.isHolonomic = isHolonomic;

        AutoBuilder.pathfindingConfigured = false;
    }

    /**
     * Configures the AutoBuilder with custom path following command builder. Building pathfinding
     * commands is not supported if using a custom command builder. Custom path following commands
     * will not have the path flipped for them, and event markers will not be triggered
     * automatically.
     *
     * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
     * @param poseSupplier a supplier for the robot's current pose
     * @param resetPose a consumer for resetting the robot's pose
     * @param isHolonomic Does the robot have a holonomic drivetrain
     */
    public static void configureCustom(
            Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            boolean isHolonomic) {
        configureCustom(
                pathFollowingCommandBuilder, poseSupplier, resetPose, () -> false, isHolonomic);
    }

    /**
     * Returns whether the AutoBuilder has been configured.
     *
     * @return true if the AutoBuilder has been configured, false otherwise
     */
    public static boolean isConfigured() {
        return configured;
    }

    /**
     * Returns whether the AutoBuilder has been configured for pathfinding.
     *
     * @return true if the AutoBuilder has been configured for pathfinding, false otherwise
     */
    public static boolean isPathfindingConfigured() {
        return pathfindingConfigured;
    }

    /**
     * Get the current robot pose
     *
     * @return Current robot pose
     */
    public static Pose2d getCurrentPose() {
        return poseSupplier.get();
    }

    /**
     * Get if a path or field position should currently be flipped
     *
     * @return True if path/positions should be flipped
     */
    public static boolean shouldFlip() {
        return shouldFlipPath.getAsBoolean();
    }

    /**
     * Builds a command to follow a path. PathPlannerLib commands will also trigger event markers
     * along the way.
     *
     * @param path the path to follow
     * @return a path following command with for the given path
     * @throws AutoBuilderException if the AutoBuilder has not been configured
     */
    public static Command followPath(PathPlannerPath path) {
        if (!isConfigured()) {
            throw new AutoBuilderException(
                    "Auto builder was used to build a path following command before being configured");
        }

        return pathFollowingCommandBuilder.apply(path);
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(
            Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        if (!isPathfindingConfigured()) {
            throw new AutoBuilderException(
                    "Auto builder was used to build a pathfinding command before being configured");
        }

        return pathfindToPoseCommandBuilder.apply(pose, constraints, goalEndVelocity);
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(
            Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return pathfindToPose(pose, constraints, goalEndVelocity.in(MetersPerSecond));
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param constraints The constraints to use while pathfinding
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
        return pathfindToPose(pose, constraints, 0);
    }

    /**
     * Build a command to pathfind to a given pose that will be flipped based on the value of the
     * path flipping supplier when this command is run. If not using a holonomic drivetrain, the
     * pose rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier
     *     returns true
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPoseFlipped(
            Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        return Commands.either(
                pathfindToPose(FlippingUtil.flipFieldPose(pose), constraints, goalEndVelocity),
                pathfindToPose(pose, constraints, goalEndVelocity),
                shouldFlipPath);
    }

    /**
     * Build a command to pathfind to a given pose that will be flipped based on the value of the
     * path flipping supplier when this command is run. If not using a holonomic drivetrain, the
     * pose rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier
     *     returns true
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPoseFlipped(
            Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return pathfindToPoseFlipped(pose, constraints, goalEndVelocity.in(MetersPerSecond));
    }

    /**
     * Build a command to pathfind to a given pose that will be flipped based on the value of the
     * path flipping supplier when this command is run. If not using a holonomic drivetrain, the
     * pose rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier
     *     returns true
     * @param constraints The constraints to use while pathfinding
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPoseFlipped(Pose2d pose, PathConstraints constraints) {
        return pathfindToPoseFlipped(pose, constraints, 0);
    }

    /**
     * Build a command to pathfind to a given pose that will also trigger an additional command when
     * the remaining time on the path is less than or equal to triggerTimeBeforeEnd seconds.
     *
     * @param pose The pose to pathfind to.
     * @param constraints The constraints to use while pathfinding.
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose.
     * @param additionalCommand The extra command to schedule near the end of the path.
     * @param triggerTimeBeforeEnd The time (in seconds) before the end of the path when the
     *     additionalCommand should run.
     * @return A command that runs the pathfinding and triggers the additional command at the right
     *     time.
     */
    public static Command pathfindToPoseWithAdditionalCommand(
            Pose2d pose,
            PathConstraints constraints,
            double goalEndVelocity,
            Command additionalCommand,
            double triggerTimeBeforeEnd,
            boolean flipPath) {
        Command pathfindCommand =
                flipPath
                        ? pathfindToPoseFlipped(pose, constraints, goalEndVelocity)
                        : pathfindToPose(pose, constraints, goalEndVelocity);

        if (!(pathfindCommand instanceof PathfindingCommand)) {
            throw new IllegalArgumentException(
                    "The returned command is not an instance of PathfindingCommand");
        }
        PathfindingCommand pfCommand = (PathfindingCommand) pathfindCommand;

        return new edu.wpi.first.wpilibj2.command.ParallelCommandGroup(
                pathfindCommand,
                new WaitUntilCommand(
                                () ->
                                        !pfCommand.getTrajectoryIsNull()
                                                && pfCommand.getRemainingTime()
                                                        <= triggerTimeBeforeEnd)
                        .andThen(additionalCommand));
    }

    /**
     * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
     * drivetrain, the pose rotation delay distance will have no effect.
     *
     * @param goalPath The path to pathfind to, then follow
     * @param pathfindingConstraints The constraints to use while pathfinding
     * @return A command to pathfind to a given path, then follow the path
     */
    public static Command pathfindThenFollowPath(
            PathPlannerPath goalPath,
            PathConstraints pathfindingConstraints,
            IPathCallback pathCallback) {
        if (!isPathfindingConfigured()) {
            throw new AutoBuilderException(
                    "Auto builder was used to build a pathfinding command before being configured");
        }

        return pathfindThenFollowPathCommandBuilder.apply(
                goalPath, pathfindingConstraints, pathCallback);
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
     * option will be Commands.none()
     *
     * @return SendableChooser populated with all autos
     */
    public static SendableChooser<Command> buildAutoChooser() {
        return buildAutoChooser("");
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project
     *
     * @param defaultAutoName The name of the auto that should be the default option. If this is an
     *     empty string, or if an auto with the given name does not exist, the default option will
     *     be Commands.none()
     * @return SendableChooser populated with all autos
     */
    public static SendableChooser<Command> buildAutoChooser(String defaultAutoName) {
        return buildAutoChooserWithOptionsModifier(defaultAutoName, (stream) -> stream);
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
     * option will be Commands.none()
     *
     * @param optionsModifier A lambda function that can be used to modify the options before they
     *     go into the AutoChooser
     * @return SendableChooser populated with all autos
     */
    public static SendableChooser<Command> buildAutoChooserWithOptionsModifier(
            Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
        return buildAutoChooserWithOptionsModifier("", optionsModifier);
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project
     *
     * @param defaultAutoName The name of the auto that should be the default option. If this is an
     *     empty string, or if an auto with the given name does not exist, the default option will
     *     be Commands.none()
     * @param optionsModifier A lambda function that can be used to modify the options before they
     *     go into the AutoChooser
     * @return SendableChooser populated with all autos
     */
    public static SendableChooser<Command> buildAutoChooserWithOptionsModifier(
            String defaultAutoName,
            Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> autoNames = getAllAutoNames();

        PathPlannerAuto defaultOption = null;
        List<PathPlannerAuto> options = new ArrayList<>();

        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
                defaultOption = auto;
            } else {
                options.add(auto);
            }
        }

        if (defaultOption == null) {
            chooser.setDefaultOption("None", Commands.none());
        } else {
            chooser.setDefaultOption(defaultOption.getName(), defaultOption);
            chooser.addOption("None", Commands.none());
        }

        optionsModifier
                .apply(options.stream())
                .forEach(auto -> chooser.addOption(auto.getName(), auto));

        return chooser;
    }

    /**
     * Get a list of all auto names in the project
     *
     * @return List of all auto names
     */
    public static List<String> getAllAutoNames() {
        File[] autoFiles =
                new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles();

        if (autoFiles == null) {
            return new ArrayList<>();
        }

        return Stream.of(autoFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".auto"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }

    /**
     * Get if AutoBuilder was configured for a holonomic drive train
     *
     * @return True if holonomic
     */
    public static boolean isHolonomic() {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException("AutoBuilder was not configured before use");
        }

        return isHolonomic;
    }

    /**
     * Builds an auto command for the given auto name.
     *
     * @param autoName the name of the auto to build
     * @return an auto command for the given auto name
     */
    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    /**
     * Create a command to reset the robot's odometry to a given blue alliance pose
     *
     * @param bluePose The pose to reset to, relative to blue alliance origin
     * @return Command to reset the robot's odometry
     */
    public static Command resetOdom(Pose2d bluePose) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException("AutoBuilder was not configured before use");
        }

        return Commands.runOnce(
                () -> {
                    boolean flip = shouldFlipPath.getAsBoolean();
                    if (flip) {
                        resetPose.accept(FlippingUtil.flipFieldPose(bluePose));
                    } else {
                        resetPose.accept(bluePose);
                    }
                });
    }

    /** Functional interface for a function that takes 3 inputs */
    @FunctionalInterface
    public interface TriFunction<In1, In2, In3, Out> {
        /**
         * Apply the inputs to this function
         *
         * @param in1 Input 1
         * @param in2 Input 2
         * @param in3 Input 3
         * @return Output
         */
        Out apply(In1 in1, In2 in2, In3 in3);
    }

    /** Functional interface for a function that takes 4 inputs */
    @FunctionalInterface
    public interface QuadFunction<In1, In2, In3, In4, Out> {
        /**
         * Apply the inputs to this function
         *
         * @param in1 Input 1
         * @param in2 Input 2
         * @param in3 Input 3
         * @param in4 Input 4
         * @return Output
         */
        Out apply(In1 in1, In2 in2, In3 in3, In4 in4);
    }

    /** Functional interface for a function that takes 5 inputs */
    @FunctionalInterface
    public interface PentaFunction<In1, In2, In3, In4, In5, Out> {
        /**
         * Apply the inputs to this function
         *
         * @param in1 Input 1
         * @param in2 Input 2
         * @param in3 Input 3
         * @param in4 Input 4
         * @param in5 Input 5
         * @return Output
         */
        Out apply(In1 in1, In2 in2, In3 in3, In4 in4, In5 in5);
    }

    /** Functional interface for a function that takes 6 inputs */
    @FunctionalInterface
    public interface HexaFunction<In1, In2, In3, In4, In5, In6, Out> {
        /**
         * Apply the inputs to this function
         *
         * @param in1 Input 1
         * @param in2 Input 2
         * @param in3 Input 3
         * @param in4 Input 4
         * @param in5 Input 5
         * @param in6 Input 6
         * @return Output
         */
        Out apply(In1 in1, In2 in2, In3 in3, In4 in4, In5 in5, In6 in6);
    }
}
