package com.team254.frc2025.controlboard;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Consumer;

public class ModalControls {
    private static Optional<ModalControls> instance = Optional.empty();

    public enum Mode {
        CORAL,
        ALGAECLIMB,
        CORALMANUAL
    }

    private Mode currentMode = Mode.CORAL;
    private Consumer<Mode> stateChangeConsumer;

    private Trigger scoreTrigger;
    private Trigger intakeTrigger;

    private Trigger setDefaultRobotWideTrigger;
    private Trigger setDefaultRobotTightTrigger;

    public static ModalControls getInstance() {
        if (instance.isEmpty()) {
            instance = Optional.of(new ModalControls());
        }
        return instance.get();
    }

    public Mode getMode() {
        return currentMode;
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.currentMode != newMode && this.stateChangeConsumer != null) {
            this.stateChangeConsumer.accept(newMode);
        }
    }

    private Trigger modeSpecific(Trigger trigger, Mode mode) {
        return trigger.and(new Trigger(() -> this.currentMode == mode));
    }

    public void forceSetMode(Mode mode) {
        maybeTriggerStateChangeConsumer(mode);
        setMode(mode);
    }

    public void configureBindings() {
        ControlBoard.getInstance()
                .getCoralMode()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    maybeTriggerStateChangeConsumer(Mode.CORAL);
                                    setMode(Mode.CORAL);
                                }));

        ControlBoard.getInstance()
                .getAlgaeClimbMode()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    maybeTriggerStateChangeConsumer(Mode.ALGAECLIMB);
                                    setMode(Mode.ALGAECLIMB);
                                }));

        ControlBoard.getInstance()
                .getCoralManualMode()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    maybeTriggerStateChangeConsumer(Mode.CORALMANUAL);
                                    setMode(Mode.CORALMANUAL);
                                }));

        scoreTrigger = ControlBoard.getInstance().score();
        intakeTrigger = ControlBoard.getInstance().intake();
        setDefaultRobotWideTrigger = ControlBoard.getInstance().setDefaultRobotWide();
        setDefaultRobotTightTrigger = ControlBoard.getInstance().setDefaultRobotTight();
    }

    private Trigger intake() {
        return intakeTrigger.and(scoreTrigger.negate());
    }

    private Trigger score() {
        return scoreTrigger.and(intakeTrigger.negate());
    }

    public Trigger setDefaultRobotWide() {
        return setDefaultRobotWideTrigger.and(setDefaultRobotTightTrigger.negate());
    }

    public Trigger setDefaultRobotTight() {
        return setDefaultRobotTightTrigger.and(setDefaultRobotWideTrigger.negate());
    }

    public Trigger algaeClimbMode() {
        return new Trigger(() -> this.currentMode == Mode.ALGAECLIMB);
    }

    public Trigger coralMode() {
        return new Trigger(() -> this.currentMode == Mode.CORAL);
    }

    public Trigger coralManualMode() {
        return new Trigger(() -> this.currentMode == Mode.CORALMANUAL);
    }

    /////////////// ALGAE CLIMB MODE ///////////////

    public Trigger autoAlignReefIntake() {
        return modeSpecific(ControlBoard.getInstance().autoAlignReefIntake(), Mode.ALGAECLIMB);
    }

    public Trigger groundIntakeAlgae() {
        return modeSpecific(intake(), Mode.ALGAECLIMB);
    }

    public Trigger lollipopIntake() {
        return modeSpecific(ControlBoard.getInstance().lollipopIntake(), Mode.ALGAECLIMB);
    }

    public Trigger bargeManualStage() {
        return modeSpecific(ControlBoard.getInstance().bargeManualStage(), Mode.ALGAECLIMB);
    }

    public Trigger processorManualStage() {
        return modeSpecific(ControlBoard.getInstance().processorManualStage(), Mode.ALGAECLIMB);
    }

    public Trigger bargeAutoAlignStage() {
        return modeSpecific(ControlBoard.getInstance().povUp(), Mode.ALGAECLIMB);
    }

    public Trigger processorAutoAlignStage() {
        return modeSpecific(ControlBoard.getInstance().povDown(), Mode.ALGAECLIMB);
    }

    public Trigger scoreAlgae() {
        return modeSpecific(score(), Mode.ALGAECLIMB);
    }

    public Trigger stageAlgaeL2() {
        return modeSpecific(ControlBoard.getInstance().reefIntakeAlgae(), Mode.ALGAECLIMB);
    }

    public Trigger stageAlgaeL3() {
        return modeSpecific(ControlBoard.getInstance().scoreBarge(), Mode.ALGAECLIMB);
    }

    public Trigger manualIntakeAlgae() {
        return modeSpecific(ControlBoard.getInstance().manualIntakeAlgae(), Mode.ALGAECLIMB);
    }

    public Trigger moveToAlgaeLevel() {
        return modeSpecific(ControlBoard.getInstance().score(), Mode.ALGAECLIMB);
    }

    public Trigger deployClimber() {
        return modeSpecific(ControlBoard.getInstance().povLeft(), Mode.ALGAECLIMB);
    }

    public Trigger climb() {
        return modeSpecific(ControlBoard.getInstance().povRight(), Mode.ALGAECLIMB);
    }

    public Trigger coralIntakeToIndexer() {
        return modeSpecific(ControlBoard.getInstance().rightStick(), Mode.ALGAECLIMB);
    }

    public Trigger stowAlgae() {
        return modeSpecific(ControlBoard.getInstance().stow(), Mode.ALGAECLIMB);
    }

    /////////////// CORAL MODE ///////////////

    public Trigger leftBranchSelect() {
        return modeSpecific(ControlBoard.getInstance().autoAlignLeft(), Mode.CORAL);
    }

    public Trigger rightBranchSelect() {
        return modeSpecific(ControlBoard.getInstance().autoAlignRight(), Mode.CORAL);
    }

    public Trigger stageL1() {
        return modeSpecific(ControlBoard.getInstance().stageL1(), Mode.CORAL);
    }

    public Trigger stageL2() {
        return modeSpecific(ControlBoard.getInstance().stageL2(), Mode.CORAL);
    }

    public Trigger stageL3() {
        return modeSpecific(ControlBoard.getInstance().stageL3(), Mode.CORAL);
    }

    public Trigger stageL4() {
        return modeSpecific(ControlBoard.getInstance().stageL4(), Mode.CORAL);
    }

    public Trigger groundCoralIntake() {
        return modeSpecific(intake(), Mode.CORAL);
    }

    public Trigger stageOrScoreCoral() {
        return modeSpecific(score(), Mode.CORAL);
    }

    public Trigger autoHP() {
        return modeSpecific(ControlBoard.getInstance().autoAlignFeeder(), Mode.CORAL);
    }

    public Trigger povAutoAlignStage() {
        return new Trigger(() -> false);
    }

    public Trigger funnelIntakeCoral() {
        return modeSpecific(ControlBoard.getInstance().intakeFunnel(), Mode.CORAL);
    }

    public Trigger coralExhaust() {
        return modeSpecific(ControlBoard.getInstance().exhaust(), Mode.CORAL);
    }

    public Trigger stowCoral() {
        return modeSpecific(ControlBoard.getInstance().stow(), Mode.CORAL);
    }

    public Trigger closestFaceAlign() {
        return modeSpecific(ControlBoard.getInstance().rightStick(), Mode.CORAL);
    }

    /////////////// CORAL MANUAL MODE ///////////////

    public Trigger stowCoralManual() {
        return modeSpecific(ControlBoard.getInstance().stow(), Mode.CORALMANUAL);
    }

    public Trigger groundCoralIntakeManual() {
        return modeSpecific(intake(), Mode.CORALMANUAL);
    }

    public Trigger groundIntakeDeployManual() {
        return modeSpecific(
                ControlBoard.getInstance().groundIntakeDeployManual(), Mode.CORALMANUAL);
    }

    public Trigger groundIntakeDeploySpinManual() {
        return modeSpecific(
                ControlBoard.getInstance().groundIntakeDeploySpinManual(), Mode.CORALMANUAL);
    }

    public Trigger coralExhaustManual() {
        return modeSpecific(ControlBoard.getInstance().exhaust(), Mode.CORALMANUAL);
    }

    public Trigger stageL1Manual() {
        return modeSpecific(ControlBoard.getInstance().stageL1(), Mode.CORALMANUAL);
    }

    public Trigger stageL2Manual() {
        return modeSpecific(ControlBoard.getInstance().stageL2(), Mode.CORALMANUAL);
    }

    public Trigger stageL3Manual() {
        return modeSpecific(ControlBoard.getInstance().stageL3(), Mode.CORALMANUAL);
    }

    public Trigger stageL4Manual() {
        return modeSpecific(ControlBoard.getInstance().stageL4(), Mode.CORALMANUAL);
    }

    public Trigger stageOrScoreCoralManual() {
        return modeSpecific(score(), Mode.CORALMANUAL);
    }

    public Trigger descoreManual() {
        return modeSpecific(ControlBoard.getInstance().descoreManual(), Mode.CORALMANUAL);
    }

    public Trigger funnelIntakeCoralManual() {
        return modeSpecific(ControlBoard.getInstance().intakeFunnel(), Mode.CORALMANUAL);
    }
}
