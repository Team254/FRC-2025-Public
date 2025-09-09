package com.team254.frc2025.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IButtonControlBoard {
    Trigger getWantToXWheels();

    Trigger getWantToAutoAlign();

    Trigger score();

    Trigger scoreBarge();

    Trigger stow();

    Trigger intake();

    Trigger intakeFunnel();

    Trigger exhaust();

    Trigger climb();

    Trigger stageL1();

    Trigger stageL2();

    Trigger stageL3();

    Trigger stageL4();

    Trigger getCoralMode();

    Trigger getAlgaeClimbMode();

    Trigger getCoralManualMode();

    Trigger autoAlignReefIntake();

    Trigger autoAlignLeft();

    Trigger groundIntakeDeployManual();

    Trigger groundIntakeDeploySpinManual();

    Trigger descoreManual();

    Trigger autoAlignRight();

    Trigger leftStick();

    Trigger rightStick();

    Trigger povUp();

    Trigger povDown();

    Trigger povLeft();

    Trigger povRight();

    Trigger reefIntakeAlgae();

    Trigger manualIntakeAlgae();

    Trigger bargeManualStage();

    Trigger processorManualStage();

    Trigger autoAlignFeeder();

    Trigger setDefaultRobotWide();

    Trigger setDefaultRobotTight();

    Trigger lollipopIntake();

    void setRumble(boolean rumble);
}
