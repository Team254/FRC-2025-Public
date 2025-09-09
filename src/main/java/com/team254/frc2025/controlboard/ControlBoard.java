package com.team254.frc2025.controlboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final IDriveControlBoard driveControlBoard;
    private final IButtonControlBoard buttonControlBoard;

    private ControlBoard() {
        driveControlBoard = GamepadDriveControlBoard.getInstance();
        buttonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return driveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return driveControlBoard.getRotation();
    }

    @Override
    public double getRotationY() {
        return driveControlBoard.getRotationY();
    }

    @Override
    public Trigger resetGyro() {
        return driveControlBoard.resetGyro();
    }

    @Override
    public Trigger getWantToXWheels() {
        return buttonControlBoard.getWantToXWheels();
    }

    @Override
    public Trigger getWantToAutoAlign() {
        return buttonControlBoard.getWantToAutoAlign();
    }

    @Override
    public Trigger autoAlignReefIntake() {
        return buttonControlBoard.autoAlignReefIntake();
    }

    @Override
    public Trigger manualIntakeAlgae() {
        return buttonControlBoard.manualIntakeAlgae();
    }

    @Override
    public Trigger score() {
        return buttonControlBoard.score();
    }

    @Override
    public Trigger scoreBarge() {
        return buttonControlBoard.scoreBarge();
    }

    @Override
    public Trigger stow() {
        return buttonControlBoard.stow();
    }

    @Override
    public Trigger intake() {
        return buttonControlBoard.intake();
    }

    @Override
    public Trigger intakeFunnel() {
        return buttonControlBoard.intakeFunnel();
    }

    @Override
    public Trigger exhaust() {
        return buttonControlBoard.exhaust();
    }

    @Override
    public Trigger stageL1() {
        return buttonControlBoard.stageL1();
    }

    @Override
    public Trigger stageL2() {
        return buttonControlBoard.stageL2();
    }

    @Override
    public Trigger stageL3() {
        return buttonControlBoard.stageL3();
    }

    @Override
    public Trigger stageL4() {
        return buttonControlBoard.stageL4();
    }

    @Override
    public Trigger climb() {
        return buttonControlBoard.climb();
    }

    @Override
    public Trigger getCoralMode() {
        return buttonControlBoard.getCoralMode();
    }

    @Override
    public Trigger getAlgaeClimbMode() {
        return buttonControlBoard.getAlgaeClimbMode();
    }

    @Override
    public Trigger getCoralManualMode() {
        return buttonControlBoard.getCoralManualMode();
    }

    @Override
    public Trigger autoAlignLeft() {
        return buttonControlBoard.autoAlignLeft();
    }

    @Override
    public Trigger autoAlignRight() {
        return buttonControlBoard.autoAlignRight();
    }

    @Override
    public Trigger leftStick() {
        return buttonControlBoard.leftStick();
    }

    @Override
    public Trigger rightStick() {
        return buttonControlBoard.rightStick();
    }

    @Override
    public Trigger povUp() {
        return buttonControlBoard.povUp();
    }

    @Override
    public Trigger reefIntakeAlgae() {
        return buttonControlBoard.reefIntakeAlgae();
    }

    @Override
    public Trigger bargeManualStage() {
        return buttonControlBoard.bargeManualStage();
    }

    @Override
    public Trigger processorManualStage() {
        return buttonControlBoard.processorManualStage();
    }

    @Override
    public Trigger povDown() {
        return buttonControlBoard.povDown();
    }

    @Override
    public Trigger povLeft() {
        return buttonControlBoard.povLeft();
    }

    @Override
    public Trigger povRight() {
        return buttonControlBoard.povRight();
    }

    @Override
    public Trigger setDefaultRobotWide() {
        return buttonControlBoard.setDefaultRobotWide();
    }

    @Override
    public Trigger setDefaultRobotTight() {
        return buttonControlBoard.setDefaultRobotTight();
    }

    @Override
    public Trigger autoAlignFeeder() {
        return buttonControlBoard.autoAlignFeeder();
    }

    @Override
    public Trigger groundIntakeDeployManual() {
        return buttonControlBoard.groundIntakeDeployManual();
    }

    @Override
    public Trigger groundIntakeDeploySpinManual() {
        return buttonControlBoard.groundIntakeDeploySpinManual();
    }

    @Override
    public Trigger descoreManual() {
        return buttonControlBoard.descoreManual();
    }

    @Override
    public Trigger lollipopIntake() {
        return buttonControlBoard.lollipopIntake();
    }

    @Override
    public void setRumble(boolean rumble) {
        buttonControlBoard.setRumble(rumble);
    }

    public Command rumble() {
        return Commands.startEnd(() -> setRumble(true), () -> setRumble(false));
    }
}
