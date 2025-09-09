package com.team254.lib.util;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/** A simulated Xbox controller that uses a provided ControllerMapping. */
public class SimXboxController extends XboxController {
    protected final ControllerMapping mapping;

    /**
     * Constructs an instance using the provided port and mapping.
     *
     * @param port The port index on the Driver Station.
     * @param mapping The mapping of button/axis names to raw values.
     */
    public SimXboxController(final int port, ControllerMapping mapping) {
        super(port);
        this.mapping = mapping;
        HAL.report(FRCNetComm.tResourceType.kResourceType_XboxController, port + 1);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(mapping.getAxis("LeftX"));
    }

    @Override
    public double getRightX() {
        return getRawAxis(mapping.getAxis("RightX"));
    }

    @Override
    public double getLeftY() {
        return getRawAxis(mapping.getAxis("LeftY"));
    }

    @Override
    public double getRightY() {
        return getRawAxis(mapping.getAxis("RightY"));
    }

    @Override
    public double getLeftTriggerAxis() {
        return getRawAxis(mapping.getAxis("LeftTrigger"));
    }

    @Override
    public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(mapping.getAxis("LeftTrigger"), threshold, loop);
    }

    @Override
    public BooleanEvent leftTrigger(EventLoop loop) {
        return leftTrigger(0.5, loop);
    }

    @Override
    public double getRightTriggerAxis() {
        return getRawAxis(mapping.getAxis("RightTrigger"));
    }

    @Override
    public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(mapping.getAxis("RightTrigger"), threshold, loop);
    }

    @Override
    public BooleanEvent rightTrigger(EventLoop loop) {
        return rightTrigger(0.5, loop);
    }

    @Override
    public boolean getAButton() {
        return getRawButton(mapping.getButton("A"));
    }

    @Override
    public boolean getAButtonPressed() {
        return getRawButtonPressed(mapping.getButton("A"));
    }

    @Override
    public boolean getAButtonReleased() {
        return getRawButtonReleased(mapping.getButton("A"));
    }

    @Override
    public BooleanEvent a(EventLoop loop) {
        return button(mapping.getButton("A"), loop);
    }

    @Override
    public boolean getBButton() {
        return getRawButton(mapping.getButton("B"));
    }

    @Override
    public boolean getBButtonPressed() {
        return getRawButtonPressed(mapping.getButton("B"));
    }

    @Override
    public boolean getBButtonReleased() {
        return getRawButtonReleased(mapping.getButton("B"));
    }

    @Override
    public BooleanEvent b(EventLoop loop) {
        return button(mapping.getButton("B"), loop);
    }

    @Override
    public boolean getXButton() {
        return getRawButton(mapping.getButton("X"));
    }

    @Override
    public boolean getXButtonPressed() {
        return getRawButtonPressed(mapping.getButton("X"));
    }

    @Override
    public boolean getXButtonReleased() {
        return getRawButtonReleased(mapping.getButton("X"));
    }

    @Override
    public BooleanEvent x(EventLoop loop) {
        return button(mapping.getButton("X"), loop);
    }

    @Override
    public boolean getYButton() {
        return getRawButton(mapping.getButton("Y"));
    }

    @Override
    public boolean getYButtonPressed() {
        return getRawButtonPressed(mapping.getButton("Y"));
    }

    @Override
    public boolean getYButtonReleased() {
        return getRawButtonReleased(mapping.getButton("Y"));
    }

    @Override
    public BooleanEvent y(EventLoop loop) {
        return button(mapping.getButton("Y"), loop);
    }

    @Override
    public boolean getLeftBumperButton() {
        return getRawButton(mapping.getButton("LeftBumper"));
    }

    @Override
    public boolean getLeftBumperButtonPressed() {
        return getRawButtonPressed(mapping.getButton("LeftBumper"));
    }

    @Override
    public boolean getLeftBumperButtonReleased() {
        return getRawButtonReleased(mapping.getButton("LeftBumper"));
    }

    @Override
    public BooleanEvent leftBumper(EventLoop loop) {
        return button(mapping.getButton("LeftBumper"), loop);
    }

    @Override
    public boolean getRightBumperButton() {
        return getRawButton(mapping.getButton("RightBumper"));
    }

    @Override
    public boolean getRightBumperButtonPressed() {
        return getRawButtonPressed(mapping.getButton("RightBumper"));
    }

    @Override
    public boolean getRightBumperButtonReleased() {
        return getRawButtonReleased(mapping.getButton("RightBumper"));
    }

    @Override
    public BooleanEvent rightBumper(EventLoop loop) {
        return button(mapping.getButton("RightBumper"), loop);
    }

    @Override
    public boolean getBackButton() {
        return getRawButton(mapping.getButton("Back"));
    }

    @Override
    public boolean getBackButtonPressed() {
        return getRawButtonPressed(mapping.getButton("Back"));
    }

    @Override
    public boolean getBackButtonReleased() {
        return getRawButtonReleased(mapping.getButton("Back"));
    }

    @Override
    public BooleanEvent back(EventLoop loop) {
        return button(mapping.getButton("Back"), loop);
    }

    @Override
    public boolean getStartButton() {
        return getRawButton(mapping.getButton("Start"));
    }

    @Override
    public boolean getStartButtonPressed() {
        return getRawButtonPressed(mapping.getButton("Start"));
    }

    @Override
    public boolean getStartButtonReleased() {
        return getRawButtonReleased(mapping.getButton("Start"));
    }

    @Override
    public BooleanEvent start(EventLoop loop) {
        return button(mapping.getButton("Start"), loop);
    }

    @Override
    public boolean getLeftStickButton() {
        return getRawButton(mapping.getButton("LeftStick"));
    }

    @Override
    public boolean getLeftStickButtonPressed() {
        return getRawButtonPressed(mapping.getButton("LeftStick"));
    }

    @Override
    public boolean getLeftStickButtonReleased() {
        return getRawButtonReleased(mapping.getButton("LeftStick"));
    }

    @Override
    public BooleanEvent leftStick(EventLoop loop) {
        return button(mapping.getButton("LeftStick"), loop);
    }

    @Override
    public boolean getRightStickButton() {
        return getRawButton(mapping.getButton("RightStick"));
    }

    @Override
    public boolean getRightStickButtonPressed() {
        return getRawButtonPressed(mapping.getButton("RightStick"));
    }

    @Override
    public boolean getRightStickButtonReleased() {
        return getRawButtonReleased(mapping.getButton("RightStick"));
    }

    @Override
    public BooleanEvent rightStick(EventLoop loop) {
        return button(mapping.getButton("RightStick"), loop);
    }
}
