package com.team254.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class StateTransition {
    private final SuperstructureState fromState;
    private final SuperstructureState toState;
    private final Supplier<Command> commandSupplier;
    private final double transitionTime; // time in seconds for the transition
    private final boolean collision; // if true, this move is considered unsafe

    public StateTransition(
            SuperstructureState fromState,
            SuperstructureState toState,
            Supplier<Command> commandSupplier,
            double transitionTime,
            boolean collision) {
        this.fromState = fromState;
        this.toState = toState;
        this.commandSupplier = commandSupplier;
        this.transitionTime = transitionTime;
        this.collision = collision;
    }

    public SuperstructureState getFromState() {
        return fromState;
    }

    public SuperstructureState getToState() {
        return toState;
    }

    public Command getCommand() {
        return commandSupplier.get();
    }

    public double getTransitionTime() {
        return transitionTime;
    }

    public boolean hasCollision() {
        return collision;
    }

    @Override
    public String toString() {
        return "Transition from "
                + fromState
                + " to "
                + toState
                + " (time: "
                + transitionTime
                + ", collision: "
                + collision
                + ")";
    }
}
