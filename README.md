# FRC-2025-Public
Team 254's 2025 FRC robot code for [Undertow](https://www.team254.com/first/2025/). Our robot's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 17 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 17 directory

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details

### Simulation
* To simulate the robot run `./gradlew simulateJavaRelease`
* You will need to configure your keyboard settings for 1 Xbox Controller.
* To set up 3d visualization in AdvantageScope, import the `2025 AdvantageScope Layout.json`.
  * If the JSON import does not work, you need to set up a 3D tab as follows:
    * 3D Poses: Robot (components) : NT: "ComponentsPoseArray"
    * 3D Poses: Robot : NT: "Drive/Viz/Pose3d"
    * 3D Poses: Coral : NT: "Sim/HeldCoral/Pose3d"
    * 3D Poses: Algae : NT: "Sim/HeldAlgae/Pose3d"

## Code Highlights
* Modal Control System

	The robot uses a sophisticated modal control system to adapt behavior based on the current game objective. The system includes multiple modes for handling both coral and algae game pieces:
  - CORAL - Ground and Human Player station coral intake, automatic staging and scoring at reef branches
  - ALGAECLIMB - Algae intake and processing, with climbing functionality for end-game
  - CORALMANUAL - Manual coral control for precise positioning and recovery scenarios
  
  The [SuperstructureStateMachine](src/main/java/com/team254/frc2025/subsystems/superstructure/SuperstructureStateMachine.java) coordinates all subsystems to achieve complex multi-step actions while the [ModalControls](src/main/java/com/team254/frc2025/controlboard/ModalControls.java) system allows drivers to seamlessly switch between different robot behaviors.

* Intelligent Pathfinding Autonomous

	The robot features advanced pathfinding autonomous capabilities using A* pathfinding with dynamic obstacle avoidance. The [PathfindingAuto](src/main/java/com/team254/frc2025/auto/PathfindingAuto.java) system allows for:
  - Dynamic route planning around field obstacles (reefs, coral stations, other robots)
  - [Configurable autonomous sequences](src/main/java/com/team254/frc2025/auto/AutoModeSelector.java) with multiple starting positions and scoring strategies
  - Real-time path recalculation based on game piece acquisition success
  - Feeder strategy selection for coordinated alliance play
  
  The autonomous uses pre-generated navigation grids and supports multiple game piece priorities and scoring levels, adapting to match conditions in real-time.

* Advanced Coral and Algae Handling

	Our robot precisely handles both coral and algae game pieces with sophisticated state tracking:
  - [CoralStateTracker](src/main/java/com/team254/frc2025/subsystems/superstructure/CoralStateTracker.java) monitors coral position throughout the robot (intake → indexer → claw → staged)
  - Automatic scoring height selection (L1-L4) based on reef branch availability
  - Precise algae processing and barge/processor scoring
  - Coordinated intake, indexer, and claw subsystems for reliable game piece control
  
  The system includes comprehensive sensor integration and state machine logic to handle complex game piece interactions and scoring scenarios.

* AdvantageScope Simulation & Logging

	We use AdvantageScope extensively for both simulation and match analysis:
  - Full robot 3D visualization with articulating mechanisms (claw, wrist, elevator, intake)
  - Real-time coral and algae tracking throughout the robot systems
  - Comprehensive logging via AdvantageKit for post-match analysis
  - Simulated game piece physics for intake and scoring validation
  
  The [SimulatedRobotState](src/main/java/com/team254/frc2025/simulation/SimulatedRobotState.java) provides accurate game piece simulation including coral funnel behavior, reef branch contact detection, and climbing simulation.

* Precision Auto-Alignment

	The robot features multiple auto-alignment capabilities for accurate scoring:
  - [Auto-align to reef branches](src/main/java/com/team254/frc2025/commands/AutoAlignToPoseCommand.java) for precise coral placement
  - Feeder station alignment for Human Player coral intake
  - Processor and barge alignment for algae scoring
  - Dynamic target selection based on field position and alliance strategy

## Package Functions
- [`com.team254.frc2025`](src/main/java/com/team254/frc2025)

	Contains the robot's central functions and holds a class with all numerical constants used throughout the code (see [`Constants.java`](src/main/java/com/team254/frc2025/Constants.java)). The [`RobotContainer`](src/main/java/com/team254/frc2025/RobotContainer.java) class controls all routines depending on the robot mode, while the [`RobotState`](src/main/java/com/team254/frc2025/RobotState.java) class tracks the current position of the robot and game pieces.

- [`com.team254.frc2025.auto`](src/main/java/com/team254/frc2025/auto)

	Contains autonomous mode selection and pathfinding autonomous commands. Includes sophisticated auto mode selection with starting positions, feeder strategies, and scoring sequences.

- [`com.team254.frc2025.commands`](src/main/java/com/team254/frc2025/commands)

	Contains autonomous commands, drive commands, and other complex robot behaviors including auto-alignment and manual control commands.

- [`com.team254.frc2025.controlboard`](src/main/java/com/team254/frc2025/controlboard)

	Contains driver control interfaces including modal controls for switching between coral, algae, and manual modes. Supports both joystick and gamepad configurations.

- [`com.team254.frc2025.factories`](src/main/java/com/team254/frc2025/factories)

	Contains command factories for complex subsystem coordination, including superstructure state management and auto-alignment functionality.

- [`com.team254.frc2025.simulation`](src/main/java/com/team254/frc2025/simulation)

	Contains simulation code for robot state, game piece physics, and field interaction modeling used in AdvantageScope visualization.

- [`com.team254.frc2025.subsystems`](src/main/java/com/team254/frc2025/subsystems)

	Contains code for all robot subsystems including drive, intake, indexer, claw, wrist, elevator, climber, and vision. Each subsystem implements the AdvantageKit IO pattern with separate hardware and simulation interfaces.

- [`com.team254.frc2025.utils`](src/main/java/com/team254/frc2025/utils)

	Contains utility classes for swerve drive simulation and other robot-specific helper functions.

- [`com.team254.frc2025.viz`](src/main/java/com/team254/frc2025/viz)

	Contains visualization code for AdvantageScope including robot pose visualization and reef structure rendering.

- [`com.team254.lib.commands`](src/main/java/com/team254/lib/commands)

	Contains enhanced command utilities including ChezyRepeatCommand and ChezySequenceCommandGroup for complex command composition.

- [`com.team254.lib.drivers`](src/main/java/com/team254/lib/drivers)

	Contains CAN device utilities and hardware abstraction layers including CANDeviceId for device identification.

- [`com.team254.lib.limelight`](src/main/java/com/team254/lib/limelight)

	Contains Limelight camera integration for vision-based localization and targeting, including LimelightHelpers utility class.

- [`com.team254.lib.pathplanner`](src/main/java/com/team254/lib/pathplanner)

	Contains enhanced PathPlanner integration with custom A* pathfinding, improved PID control, and advanced trajectory following capabilities.

- [`com.team254.lib.reefscape`](src/main/java/com/team254/lib/reefscape)

	Contains 2025 Reefscape game-specific utilities including reef branch calculations and scoring location management.

- [`com.team254.lib.subsystems`](src/main/java/com/team254/lib/subsystems)

	Contains base subsystem templates and motor control abstractions used throughout the robot code, including servo motor subsystems with CANCoder support.

- [`com.team254.lib.time`](src/main/java/com/team254/lib/time)

	Contains robot timing utilities for consistent time measurement across different execution contexts.

- [`com.team254.lib.util`](src/main/java/com/team254/lib/util)

	Contains utility classes including field constants, mathematical helpers, CAN logging, and various robot-specific calculations.

## Additional Notes

### Package Structure
The codebase follows a clean separation between robot-specific code (`com.team254.frc2025.*`) and reusable library code (`com.team254.lib.*`). The library packages provide abstractions and utilities that could be reused across different robot projects.

### Build System
This project uses Gradle with WPILib's GradleRIO plugin version 2025.3.2. The build system automatically handles dependency management for FRC libraries and vendor dependencies.

## Variable Naming Conventions
- k[A-Z]*** (i.e. `kDriveWheelbaseMeters`): Final constants, especially those found in the [`Constants.java`](src/main/java/com/team254/frc2025/Constants.java) file. Constants follow the pattern of lowercase 'k' followed by a capital letter and descriptive name in camelCase.

## Licenses
- Team 254 code: MIT — see `LICENSE`.
- WPILib: BSD — see `WPILib-License.md`.
- PathPlanner: MIT — see `PathPlanner-License.md`.