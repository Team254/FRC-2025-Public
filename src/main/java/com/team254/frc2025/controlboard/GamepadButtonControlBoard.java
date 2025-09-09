package com.team254.frc2025.controlboard;

import com.team254.frc2025.Constants;
import com.team254.frc2025.Robot;
import com.team254.lib.util.CommandSimXboxController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard instance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadButtonControlBoard();
        }
        return instance;
    }

    private final CommandXboxController controller;

    @SuppressWarnings("unused")
    private final CommandXboxController additionalController;

    @SuppressWarnings("unused")
    private GamepadButtonControlBoard() {
        if (Constants.kForceDriveGamepad
                || DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort)) {
            if (Robot.isSimulation()) {
                controller = new CommandSimXboxController(Constants.kDriveGamepadPort);
            } else {
                controller = new CommandXboxController(Constants.kDriveGamepadPort);
            }
            additionalController =
                    new CommandXboxController(Constants.kGamepadAdditionalControllerPort);
        } else {
            controller = new CommandXboxController(Constants.kOperatorControllerPort);
        }
    }

    @Override
    public Trigger getWantToXWheels() {
        return controller.start().and(controller.back().negate());
    }

    @Override
    public Trigger getWantToAutoAlign() {
        return controller.start();
    }

    @Override
    public Trigger score() {
        return controller.rightTrigger();
    }

    @Override
    public Trigger scoreBarge() {
        return controller.b();
    }

    @Override
    public Trigger reefIntakeAlgae() {
        return controller.a();
    }

    @Override
    public Trigger bargeManualStage() {
        return controller.y();
    }

    @Override
    public Trigger processorManualStage() {
        return controller.x();
    }

    @Override
    public Trigger stow() {
        return controller.leftStick();
    }

    @Override
    public Trigger intake() {
        return controller.leftTrigger();
    }

    @Override
    public Trigger intakeFunnel() {
        return controller.povLeft();
    }

    @Override
    public Trigger exhaust() {
        return controller.povRight();
    }

    @Override
    public Trigger climb() {
        return controller.rightBumper();
    }

    @Override
    public Trigger stageL1() {
        return controller.x();
    }

    @Override
    public Trigger stageL2() {
        return controller.a();
    }

    @Override
    public Trigger stageL3() {
        return controller.b();
    }

    @Override
    public Trigger stageL4() {
        return controller.y();
    }

    @Override
    public Trigger getCoralMode() {
        return controller.start().and(controller.back().negate()).debounce(0.1);
    }

    @Override
    public Trigger getAlgaeClimbMode() {
        return controller.back().and(controller.start().negate()).debounce(0.1);
    }

    @Override
    public Trigger getCoralManualMode() {
        return controller.back().and(controller.start()).debounce(0.5, DebounceType.kBoth);
    }

    @Override
    public Trigger autoAlignReefIntake() {
        return controller.leftBumper();
    }

    @Override
    public Trigger manualIntakeAlgae() {
        return controller.rightStick();
    }

    @Override
    public Trigger autoAlignLeft() {
        return controller.leftBumper();
    }

    @Override
    public Trigger groundIntakeDeployManual() {
        return controller.povUp();
    }

    @Override
    public Trigger descoreManual() {
        return controller.rightBumper();
    }

    @Override
    public Trigger groundIntakeDeploySpinManual() {
        return controller.povDown();
    }

    @Override
    public Trigger autoAlignRight() {
        return controller.rightBumper();
    }

    @Override
    public Trigger leftStick() {
        return controller.leftStick();
    }

    @Override
    public Trigger rightStick() {
        return controller.rightStick();
    }

    @Override
    public Trigger povUp() {
        return controller.povUp();
    }

    @Override
    public Trigger povDown() {
        return controller.povDown();
    }

    @Override
    public Trigger povLeft() {
        return controller.povLeft();
    }

    @Override
    public Trigger povRight() {
        return controller.povRight();
    }

    @Override
    public Trigger autoAlignFeeder() {
        return controller.rightStick().debounce(Constants.kPOVDebounceTimeSeconds);
    }

    @Override
    public Trigger setDefaultRobotWide() {
        return controller.povDown();
    }

    @Override
    public Trigger setDefaultRobotTight() {
        return controller.povUp();
    }

    @Override
    public Trigger lollipopIntake() {
        return controller.rightBumper();
    }

    @Override
    public void setRumble(boolean rumble) {
        controller.getHID().setRumble(RumbleType.kBothRumble, rumble ? 1 : 0);
    }
}
